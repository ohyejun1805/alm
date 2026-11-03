#!/usr/bin/env python3
"""Drive /cmd_vel from the camera-derived error, then along the rail on wheel odometry.

The simulator's true pose is never read here. The loop runs on the wall clock so it keeps
publishing zero velocity even when the simulation clock stops along with the camera.
"""
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from rail_align_demo import controller as ctl
from rail_align_demo.msg import AlignmentError


class ControllerNode(object):
    def __init__(self):
        cfg = rospy.get_param('/rail_align')
        start_pose = cfg['start_poses'].get(rospy.get_param('~start', ''), {})
        self.controller = ctl.AlignmentController(cfg, start_pose.get('search_direction', 0))
        self.detect_timeout = float(cfg['safety']['detect_timeout_s'])
        self.image_timeout = float(cfg['safety']['image_timeout_s'])
        self.period = 1.0 / float(cfg['control']['rate_hz'])
        self.lock = threading.Lock()
        self.last_msg = None
        self.last_msg_wall = None
        self.last_valid = None
        self.last_valid_wall = None
        self.odom = None
        self.odom_wall = None
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.state_pub = rospy.Publisher('/rail_align/state', String, queue_size=5, latch=True)
        rospy.Subscriber('/rail_align/errors', AlignmentError, self.on_error, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.on_odom, queue_size=1)
        self.last_state = None
        self.last_stop_reason = None

    def on_error(self, msg):
        now = time.monotonic()
        with self.lock:
            self.last_msg = msg
            self.last_msg_wall = now
            if msg.valid:
                self.last_valid = msg
                self.last_valid_wall = now

    def on_odom(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
        with self.lock:
            self.odom = (pose.position.x, pose.position.y, yaw)
            self.odom_wall = time.monotonic()

    def fresh_odom(self):
        with self.lock:
            if self.odom is None or time.monotonic() - self.odom_wall > self.image_timeout:
                return None
            return self.odom

    def vision_status(self):
        """Return (usable measurement message or None, reason it is unusable)."""
        wall = time.monotonic()
        with self.lock:
            msg, msg_wall, valid, valid_wall = (
                self.last_msg, self.last_msg_wall, self.last_valid, self.last_valid_wall)
        if msg is None:
            return None, 'no_image_yet'
        sim_age = (rospy.Time.now() - msg.header.stamp).to_sec()
        if wall - msg_wall > self.image_timeout or sim_age > self.image_timeout:
            return None, 'image_stale(wall %.2fs, sim %.2fs)' % (wall - msg_wall, sim_age)
        if valid is None or wall - valid_wall > self.detect_timeout:
            return None, 'marker_pair_lost(%s)' % msg.reason
        return valid, ''

    def spin(self):
        while not rospy.is_shutdown():
            valid, reason = self.vision_status()
            measurement = None
            if valid is not None:
                measurement = (valid.lateral_error_m, math.radians(valid.heading_error_deg),
                               valid.longitudinal_error_m)
            # Losing the markers is normal on the rail; losing the camera or odometry is not.
            feed_ok = not reason.startswith(('image_stale', 'no_image'))
            odom = self.fresh_odom()
            if odom is None and not reason and self.controller.state in (ctl.SAFETY_STOP, ctl.ALIGNED):
                reason = 'odom_stale'
            v, w = self.controller.update(rospy.get_time(), measurement, feed_ok, odom)
            cmd = Twist()
            cmd.linear.x = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            self.report(valid, reason)
            time.sleep(self.period)

    def report(self, valid, reason):
        state = self.controller.state
        # The reason text carries changing timings, so only its kind decides when to log.
        kind = reason.split('(')[0]
        if kind and kind != self.last_stop_reason and state in (ctl.SAFETY_STOP, ctl.WAIT_VISION, ctl.ALIGNED):
            rospy.logwarn('zero velocity: %s', reason)
        self.last_stop_reason = kind
        if state != self.last_state:
            self.state_pub.publish(state)
            rospy.loginfo('state %s -> %s', self.last_state, state)
            if state == ctl.ALIGNED:
                rospy.loginfo('ALIGNED lateral=%+.3f m heading=%+.2f deg longitudinal=%+.3f m',
                              valid.lateral_error_m, valid.heading_error_deg,
                              valid.longitudinal_error_m)
            elif state == ctl.RAIL_END:
                rospy.loginfo('RAIL_END: rail drive finished')
            elif state == ctl.FAILED:
                rospy.logerr('FAILED: arrived off-center after %d retreats', self.controller.retreats)
            self.last_state = state


if __name__ == '__main__':
    rospy.init_node('alignment_controller')
    ControllerNode().spin()
