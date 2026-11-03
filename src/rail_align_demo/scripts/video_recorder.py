#!/usr/bin/env python3
"""Save the demo as an MP4: overview and chase views on the left, ArUco recognition on the right."""
import os
import subprocess
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rail_align_demo import controller as ctl

SMALL_SIZE = (640, 360)
MAIN_SIZE = (1280, 720)
TITLE_H = STATUS_H = 36
AFTER_FINAL_S = 2.0
STATE_NOTES = {
    ctl.WAIT_VISION: 'waiting for both markers',
    ctl.SEARCH: 'no markers in view yet: turning in place to find them',
    ctl.ALIGN_HEADING: 'turning in place toward the center line',
    ctl.APPROACH: 'driving to the stop point using camera error only',
    ctl.FINAL_HEADING: 'turning in place to the rail direction',
    ctl.HOLD: 'stopped, checking tolerances for the hold time',
    ctl.ALIGNED: 'aligned, stopped before entry',
    ctl.RAIL_DRIVE: 'on the rail: markers leave the view, driving on wheel odometry',
    ctl.RAIL_END: 'end of rail run, stopped',
    ctl.SAFETY_STOP: 'vision lost: zero velocity',
    ctl.FAILED: 'alignment failed, stopped',
}


def titled(image, title, size):
    panel = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    header = np.full((TITLE_H, size[0], 3), 40, dtype=np.uint8)
    cv2.putText(header, title, (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return np.vstack([header, panel])


class VideoRecorder(object):
    def __init__(self):
        cfg = rospy.get_param('/rail_align')
        self.final_states = ((ctl.RAIL_END, ctl.FAILED) if cfg['rail_drive']['enabled']
                             else (ctl.ALIGNED, ctl.FAILED))
        output_dir = os.path.abspath(rospy.get_param('~output_dir'))
        os.makedirs(output_dir, exist_ok=True)
        name = '%s_%s.mp4' % (rospy.get_param('~scenario', 'run'), time.strftime('%Y%m%d_%H%M%S'))
        self.path = os.path.join(output_dir, name)
        self.bridge = CvBridge()
        blank = np.zeros((SMALL_SIZE[1], SMALL_SIZE[0], 3), dtype=np.uint8)
        self.overview = blank
        self.chase = blank
        self.state = ctl.WAIT_VISION
        self.cmd = (0.0, 0.0)
        self.final_t = None
        self.closed = False
        self.rate = float(cfg['camera']['rate_hz'])
        self.first_stamp = None
        self.frames = 0
        size = '%dx%d' % (SMALL_SIZE[0] + MAIN_SIZE[0], 2 * (SMALL_SIZE[1] + TITLE_H))
        # Its own session, so a terminal Ctrl+C cannot cut the file off before it is closed.
        self.encoder = subprocess.Popen(
            ['ffmpeg', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', size,
             '-r', str(cfg['camera']['rate_hz']), '-i', '-', '-c:v', 'libx264', '-preset', 'veryfast',
             '-crf', '23', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', self.path],
            stdin=subprocess.PIPE, start_new_session=True)
        rospy.on_shutdown(self.close)
        rospy.Subscriber('/overview_cam/image_raw', Image, self.on_overview, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('/chase_cam/image_raw', Image, self.on_chase, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber('/rail_align/state', String, self.on_state, queue_size=5)
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=1)
        rospy.Subscriber('/rail_align/debug_image', Image, self.on_debug, queue_size=5, buff_size=2 ** 26)

    def on_overview(self, msg):
        self.overview = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def on_chase(self, msg):
        self.chase = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def on_state(self, msg):
        self.state = msg.data
        if msg.data in self.final_states and self.final_t is None:
            self.final_t = rospy.get_time()

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)

    def on_debug(self, msg):
        if self.closed:
            return
        if self.final_t is not None and rospy.get_time() - self.final_t > AFTER_FINAL_S:
            self.close()
            return
        status = np.full((STATUS_H, MAIN_SIZE[0], 3), 255, dtype=np.uint8)
        color = {ctl.ALIGNED: (0, 150, 0), ctl.RAIL_END: (0, 150, 0), ctl.SAFETY_STOP: (0, 0, 220),
                 ctl.FAILED: (0, 0, 220)}.get(self.state, (150, 60, 0))
        text = '%s | %s | v %+.2f m/s  w %+.2f rad/s | t %.1f s' % (
            self.state, STATE_NOTES.get(self.state, ''), self.cmd[0], self.cmd[1], msg.header.stamp.to_sec())
        cv2.putText(status, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.56, color, 2, cv2.LINE_AA)
        main = titled(self.bridge.imgmsg_to_cv2(msg, 'bgr8'),
                      'Front RGB camera: ArUco ID (green outline), corner 0 (red dot), marker pose axes', MAIN_SIZE)
        left = np.vstack([titled(self.overview, 'Gazebo overview', SMALL_SIZE),
                          titled(self.chase, 'Chase view: wheels outside the rails', SMALL_SIZE)])
        frame = np.hstack([left, np.vstack([main, status])])
        # Repeat a frame over any the subscriber dropped, so playback matches simulation time.
        stamp = msg.header.stamp.to_sec()
        if self.first_stamp is None:
            self.first_stamp = stamp
        target = int((stamp - self.first_stamp) * self.rate) + 1
        try:
            while self.frames < target:
                self.encoder.stdin.write(frame.tobytes())
                self.frames += 1
        except (BrokenPipeError, ValueError):
            # ValueError: close() ran on the shutdown thread between the check above and this write.
            self.close()

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.encoder.stdin.close()
        except BrokenPipeError:
            pass
        self.encoder.wait(timeout=30)
        rospy.loginfo('video saved: %s', self.path)


if __name__ == '__main__':
    rospy.init_node('video_recorder')
    VideoRecorder()
    rospy.spin()
