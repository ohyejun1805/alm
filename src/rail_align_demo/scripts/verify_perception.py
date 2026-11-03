#!/usr/bin/env python3
"""Check real Gazebo camera detection and error signs before any control is connected.

Run against `roslaunch rail_align_demo demo.launch control:=false record:=false`.
Teleports the cart with /gazebo/set_model_state and compares the camera estimate
with the expected sign and the true pose.
"""
import argparse
import csv
import math
import os
import sys
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from sensor_msgs.msg import Image

from rail_align_demo.geometry import rail_errors, target_x
from rail_align_demo.msg import AlignmentError


# name, x, y, yaw_deg, expected signs (lateral, heading, longitudinal); 0 = not checked
def pose_table(cfg):
    stop = target_x(cfg)
    table = [
        ('center', stop - 0.6, 0.0, 0.0, (0, 0, 1)),
        ('left_of_center', stop - 0.6, 0.20, 0.0, (1, 0, 1)),
        ('right_of_center', stop - 0.6, -0.20, 0.0, (-1, 0, 1)),
        ('nose_left', stop - 0.6, 0.0, 10.0, (0, 1, 1)),
        ('nose_right', stop - 0.6, 0.0, -10.0, (0, -1, 1)),
        ('far_from_stop', stop - 1.3, 0.0, 0.0, (0, 0, 1)),
        ('at_stop', stop, 0.0, 0.0, (0, 0, 0)),
        ('past_stop', stop + 0.10, 0.0, 0.0, (0, 0, -1)),
    ]
    for name in ('left', 'right'):
        pose = cfg['start_poses'][name]
        table.append(('start_' + name, pose['x'], pose['y'], pose['yaw_deg'],
                      (int(math.copysign(1, pose['y'])), int(math.copysign(1, pose['yaw_deg'])), 1)))
    return table


class Verifier(object):
    def __init__(self):
        self.cfg = rospy.get_param('/rail_align')
        self.bridge = CvBridge()
        self.latest_error = []
        self.frame = None
        self.raw_frame = None
        rospy.Subscriber('/rail_align/errors', AlignmentError, self.on_error, queue_size=10)
        rospy.Subscriber('/rail_align/debug_image', Image, self.on_debug, queue_size=1, buff_size=2 ** 25)
        rospy.Subscriber('/rail_cam/image_raw', Image, self.on_raw, queue_size=1, buff_size=2 ** 25)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def on_error(self, msg):
        self.latest_error.append(msg)

    def on_debug(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def on_raw(self, msg):
        self.raw_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def teleport(self, x, y, yaw_deg):
        state = ModelState()
        state.model_name = 'rail_cart'
        state.reference_frame = 'world'
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = float(self.cfg['vehicle']['wheel_radius_m']) + 0.001
        yaw = math.radians(yaw_deg)
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        if not self.set_state(state).success:
            raise RuntimeError('set_model_state failed')

    def truth(self):
        response = self.get_state('rail_cart', 'world')
        q = response.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        lat, head, lon = rail_errors(response.pose.position.x, response.pose.position.y, yaw,
                                     target_x(self.cfg))
        return lat, math.degrees(head), lon

    def sample(self, count=15, timeout=20.0):
        self.latest_error = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and len(self.latest_error) < count:
            time.sleep(0.02)
        return list(self.latest_error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results-dir', default=os.path.join(os.getcwd(), 'results'))
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('verify_perception')
    verifier = Verifier()
    os.makedirs(args.results_dir, exist_ok=True)
    tol = verifier.cfg['tolerance']
    limits = (float(tol['lateral_m']), float(tol['heading_deg']), float(tol['longitudinal_m']))

    rows = []
    failures = 0
    header = ('%-16s %6s %6s %6s | %-8s | %8s %8s %8s | %8s %8s %8s | %s'
              % ('pose', 'x', 'y', 'yaw', 'ids', 'cam_lat', 'cam_head', 'cam_long',
                 'gt_lat', 'gt_head', 'gt_long', 'verdict'))
    print(header)
    print('-' * len(header))
    for name, x, y, yaw_deg, signs in pose_table(verifier.cfg):
        verifier.teleport(x, y, yaw_deg)
        time.sleep(1.5)     # let the cart settle and the camera catch up
        samples = verifier.sample()
        valid = [m for m in samples if m.valid]
        truth = verifier.truth()
        ids = sorted(set(i for m in samples for i in m.detected_ids))
        if len(valid) < 0.8 * max(1, len(samples)) or not samples:
            cam = (float('nan'),) * 3
            verdict = 'FAIL(valid %d/%d; %s)' % (len(valid), len(samples),
                                                 samples[-1].reason if samples else 'no msgs')
        else:
            cam = tuple(np.mean([[m.lateral_error_m, m.heading_error_deg, m.longitudinal_error_m]
                                 for m in valid], axis=0))
            problems = []
            for label, value, gt, sign, limit in zip(('lat', 'head', 'long'), cam, truth, signs, limits):
                if sign != 0 and np.sign(value) != sign:
                    problems.append(label + '_sign')
                if abs(value - gt) > limit:
                    problems.append(label + '_vs_truth')
            verdict = 'PASS' if not problems else 'FAIL(%s)' % ','.join(problems)
        failures += not verdict.startswith('PASS')
        print('%-16s %6.2f %6.2f %6.1f | %-8s | %+8.3f %+8.2f %+8.3f | %+8.3f %+8.2f %+8.3f | %s'
              % ((name, x, y, yaw_deg, str(ids).replace(' ', '')) + cam + truth + (verdict,)))
        rows.append([name, x, y, yaw_deg, ids] + ['%.4f' % v for v in cam + truth] + [verdict])
        if verifier.raw_frame is not None:
            image_dir = os.path.join(args.results_dir, 'images')
            os.makedirs(image_dir, exist_ok=True)
            frame = verifier.frame if verifier.frame is not None else verifier.raw_frame
            cv2.imwrite(os.path.join(image_dir, 'perception_%s.png' % name), frame)

    with open(os.path.join(args.results_dir, 'sign_check.csv'), 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['pose', 'x', 'y', 'yaw_deg', 'ids', 'cam_lateral_m', 'cam_heading_deg',
                         'cam_longitudinal_m', 'gt_lateral_m', 'gt_heading_deg',
                         'gt_longitudinal_m', 'verdict'])
        writer.writerows(rows)
    print('RESULT: %s (%d failing poses)' % ('PASS' if failures == 0 else 'FAIL', failures))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
