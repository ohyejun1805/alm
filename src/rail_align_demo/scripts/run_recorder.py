#!/usr/bin/env python3
"""Log each run and compare the camera estimate with the Gazebo pose.

This is the only demo node that reads the simulator's true pose. It publishes
nothing, so the true pose cannot reach the controller.
"""
import csv
import math
import os
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rail_align_demo import controller as ctl
from rail_align_demo.geometry import rail_errors, target_x
from rail_align_demo.msg import AlignmentError

ROW_FIELDS = ['t_sim', 'valid', 'reason', 'cam_lateral_m', 'cam_heading_deg', 'cam_longitudinal_m',
              'gt_lateral_m', 'gt_heading_deg', 'gt_longitudinal_m', 'cmd_v', 'cmd_w', 'state']
SUMMARY_FIELDS = ['run', 'scenario', 'result',
                  'start_cam_lateral_m', 'start_cam_heading_deg', 'start_cam_longitudinal_m',
                  'start_gt_lateral_m', 'start_gt_heading_deg', 'start_gt_longitudinal_m',
                  'end_cam_lateral_m', 'end_cam_heading_deg', 'end_cam_longitudinal_m',
                  'end_gt_lateral_m', 'end_gt_heading_deg', 'end_gt_longitudinal_m',
                  'duration_s', 'cam_within_tolerance', 'gt_within_tolerance', 'end_gt_speed_mps',
                  'rail_duration_s', 'rail_end_x_m', 'rail_max_abs_lateral_m', 'rail_max_abs_heading_deg',
                  'rail_wheels_clear', 'rail_end_gt_speed_mps']


class Recorder(object):
    def __init__(self):
        cfg = rospy.get_param('/rail_align')
        self.tol = cfg['tolerance']
        self.rail_enabled = bool(cfg['rail_drive']['enabled'])
        # Largest offset from the center line that keeps both wheels outside their rails.
        self.clearance = ((float(cfg['vehicle']['wheel_separation_m']) - float(cfg['vehicle']['wheel_width_m'])) -
                          (float(cfg['rail']['gauge_m']) + float(cfg['rail']['width_m']))) / 2.0
        self.target_x = target_x(cfg)
        self.scenario = rospy.get_param('~scenario', 'run')
        self.results_dir = os.path.abspath(rospy.get_param('~results_dir'))
        self.exit_on_finish = rospy.get_param('~exit_on_finish', False)
        self.timeout = float(rospy.get_param('~timeout_s', 240.0))
        self.settle = float(rospy.get_param('~post_aligned_s', 2.0))
        self.bridge = CvBridge()

        self.run_id = '%s_%s' % (self.scenario, time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(os.path.join(self.results_dir, 'images'), exist_ok=True)
        self.csv_path = os.path.join(self.results_dir, self.run_id + '.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(ROW_FIELDS)

        self.truth = None
        self.speed = float('nan')
        self.cmd = (0.0, 0.0)
        self.state = ctl.WAIT_VISION
        self.start_row = None
        self.motion_t = None
        self.aligned_t = None
        self.end_t = None
        self.rail_t = None
        self.rail_max_lateral = 0.0
        self.rail_max_heading = 0.0
        self.truth_x = float('nan')
        self.window = []
        self.last_frame = None
        self.saved_start = False
        self.finished = False

        self.wait_for_cart()
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState, persistent=True)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=1)
        rospy.Subscriber('/rail_align/state', String, self.on_state, queue_size=5)
        rospy.Subscriber('/rail_align/errors', AlignmentError, self.on_error, queue_size=5)
        rospy.Subscriber('/rail_align/debug_image', Image, self.on_frame, queue_size=1, buff_size=2 ** 25)

    def wait_for_cart(self):
        rospy.wait_for_service('/gazebo/get_world_properties')
        properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        while not rospy.is_shutdown() and 'rail_cart' not in properties().model_names:
            time.sleep(0.2)

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        if self.motion_t is None and (msg.linear.x != 0.0 or msg.angular.z != 0.0):
            self.motion_t = rospy.get_time()

    def on_state(self, msg):
        self.state = msg.data
        now = rospy.get_time()
        if msg.data == ctl.ALIGNED and self.aligned_t is None:
            self.aligned_t = now
        if msg.data == ctl.RAIL_DRIVE and self.rail_t is None:
            self.rail_t = now
            self.save_frame('aligned')
        final = (ctl.RAIL_END, ctl.FAILED) if self.rail_enabled else (ctl.ALIGNED, ctl.FAILED)
        if msg.data in final and self.end_t is None:
            self.end_t = now

    def on_frame(self, msg):
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if not self.saved_start and self.start_row is not None:
            self.save_frame('start')
            self.saved_start = True

    def save_frame(self, label):
        if self.last_frame is not None:
            cv2.imwrite(os.path.join(self.results_dir, 'images',
                                     '%s_%s.png' % (self.run_id, label)), self.last_frame)

    def poll_truth(self):
        try:
            response = self.get_state('rail_cart', 'world')
        except rospy.ServiceException:
            return
        if not response.success:
            return
        q = response.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        lateral, heading, longitudinal = rail_errors(
            response.pose.position.x, response.pose.position.y, yaw, self.target_x)
        self.truth = (lateral, math.degrees(heading), longitudinal)
        self.speed = math.hypot(response.twist.linear.x, response.twist.linear.y)
        self.truth_x = response.pose.position.x
        if self.state == ctl.RAIL_DRIVE:
            self.rail_max_lateral = max(self.rail_max_lateral, abs(lateral))
            self.rail_max_heading = max(self.rail_max_heading, abs(math.degrees(heading)))

    def on_error(self, msg):
        if self.finished or self.truth is None:
            return
        cam = (msg.lateral_error_m, msg.heading_error_deg, msg.longitudinal_error_m)
        self.writer.writerow(
            ['%.3f' % msg.header.stamp.to_sec(), int(msg.valid), msg.reason] +
            ['%.4f' % v if msg.valid else '' for v in cam] +
            ['%.4f' % v for v in self.truth] +
            ['%.3f' % v for v in self.cmd] + [self.state])
        if msg.valid and self.start_row is None:
            self.start_row = (cam, self.truth)
            rospy.loginfo('start error (camera): lateral=%+.3f m heading=%+.2f deg longitudinal=%+.3f m',
                          *cam)
            rospy.loginfo('start error (gazebo): lateral=%+.3f m heading=%+.2f deg longitudinal=%+.3f m',
                          *self.truth)
        if msg.valid and self.state == ctl.ALIGNED:
            self.window.append((cam, self.truth, self.speed))

    def within(self, values):
        return (abs(values[0]) <= float(self.tol['lateral_m']) and
                abs(values[1]) <= float(self.tol['heading_deg']) and
                abs(values[2]) <= float(self.tol['longitudinal_m']))

    def finish(self):
        self.finished = True
        self.csv_file.close()
        self.save_frame('end')
        result = self.state if self.end_t is not None else 'TIMEOUT'
        row = {'run': self.run_id, 'scenario': self.scenario, 'result': result}
        if self.start_row is not None:
            for prefix, values in zip(('start_cam', 'start_gt'), self.start_row):
                row.update(self._error_fields(prefix, values))
        cam_ok = gt_ok = False
        if self.window:
            cam_end = np.median([w[0] for w in self.window], axis=0)
            gt_end = np.median([w[1] for w in self.window], axis=0)
            row.update(self._error_fields('end_cam', cam_end))
            row.update(self._error_fields('end_gt', gt_end))
            row['end_gt_speed_mps'] = '%.4f' % max(w[2] for w in self.window)
            cam_ok = self.within(cam_end)
            gt_ok = self.within(gt_end)
        row['cam_within_tolerance'] = cam_ok
        row['gt_within_tolerance'] = gt_ok
        if self.motion_t is not None and self.aligned_t is not None:
            row['duration_s'] = '%.1f' % (self.aligned_t - self.motion_t)
        if self.rail_t is not None and self.state == ctl.RAIL_END:
            row['rail_duration_s'] = '%.1f' % (self.end_t - self.rail_t)
            row['rail_end_x_m'] = '%.3f' % self.truth_x
            row['rail_max_abs_lateral_m'] = '%.4f' % self.rail_max_lateral
            row['rail_max_abs_heading_deg'] = '%.3f' % self.rail_max_heading
            row['rail_wheels_clear'] = self.rail_max_lateral < self.clearance
            row['rail_end_gt_speed_mps'] = '%.4f' % self.speed

        summary = os.path.join(self.results_dir, 'summary.csv')
        is_new = not os.path.exists(summary)
        with open(summary, 'a', newline='') as handle:
            writer = csv.DictWriter(handle, SUMMARY_FIELDS)
            if is_new:
                writer.writeheader()
            writer.writerow(row)
        rospy.loginfo('RUN RESULT %s', {k: row.get(k) for k in SUMMARY_FIELDS})
        rospy.loginfo('log: %s', self.csv_path)

    @staticmethod
    def _error_fields(prefix, values):
        return {prefix + '_lateral_m': '%.4f' % values[0],
                prefix + '_heading_deg': '%.3f' % values[1],
                prefix + '_longitudinal_m': '%.4f' % values[2]}

    def spin(self):
        rate = rospy.Rate(20.0)
        start = None
        while not rospy.is_shutdown():
            self.poll_truth()
            now = rospy.get_time()
            if start is None and now > 0.0 and self.start_row is not None:
                start = now
            timed_out = start is not None and now - start > self.timeout
            settled = self.end_t is not None and now - self.end_t >= self.settle
            if timed_out or settled:
                self.finish()
                break
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        if not self.finished:
            self.finish()
        if not self.exit_on_finish:
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('run_recorder')
    Recorder().spin()
