#!/usr/bin/env python3
"""Check that the cart stops when vision is lost. Run against a live demo.launch.

  occlude : cover the left marker during the approach, then uncover it
  pause   : pause physics during the approach, which stops camera images
  railpause: same as pause, but during the rail drive where the markers are out of view
  nofeed  : kill the estimator during the approach, so errors stop while the sim clock runs
  blind   : markers never visible (demo.launch start:=blind); the cart must not move
"""
import argparse
import math
import os
import sys
import time

import rosnode
import rospy
from gazebo_msgs.srv import DeleteModel, GetModelState, SpawnModel
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String
from std_srvs.srv import Empty

from rail_align_demo.msg import AlignmentError

OBSERVE_S = 3.0


class Monitor(object):
    def __init__(self):
        self.cmd = None
        self.cmd_log = []
        self.state = None
        self.error = None
        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd, queue_size=10)
        rospy.Subscriber('/rail_align/state', String, self.on_state, queue_size=10)
        rospy.Subscriber('/rail_align/errors', AlignmentError, self.on_error, queue_size=1)
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_log.append((time.monotonic(), msg.linear.x, msg.angular.z))

    def on_state(self, msg):
        self.state = msg.data

    def on_error(self, msg):
        self.error = msg

    def position(self):
        pose = self.get_state('rail_cart', 'world').pose
        return pose.position.x, pose.position.y

    def wait_moving(self, target_state='APPROACH', timeout=90.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.state == target_state and self.cmd and self.cmd[0] > 0.05:
                return
            time.sleep(0.02)
        raise RuntimeError('cart never reached %s (state=%s)' % (target_state, self.state))

    def wait_zero(self, since, timeout=5.0):
        """Return seconds from `since` until zero velocity was first commanded."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for stamp, v, w in self.cmd_log:
                if stamp >= since and v == 0.0 and w == 0.0:
                    return stamp - since
            time.sleep(0.01)
        return None

    def observe_stopped(self):
        """Hold for OBSERVE_S and return (all commands zero, drift in metres)."""
        start_pos = self.position()
        begin = time.monotonic()
        time.sleep(OBSERVE_S)
        window = [c for c in self.cmd_log if c[0] >= begin]
        end_pos = self.position()
        drift = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])
        return bool(window) and all(c[1] == 0.0 and c[2] == 0.0 for c in window), drift

    def wait_resumed(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.cmd and (self.cmd[0] != 0.0 or self.cmd[1] != 0.0):
                return True
            time.sleep(0.02)
        return False


def occluder_sdf(cfg):
    marker = cfg['marker']
    size = float(marker['size_m']) * 2.0
    return """<?xml version="1.0"?><sdf version="1.6"><model name="occluder"><static>true</static>
<link name="link"><visual name="v"><geometry><box><size>0.01 %f %f</size></box></geometry></visual></link>
</model></sdf>""" % (size, size)


def check_occlude(monitor, cfg):
    monitor.wait_moving()
    marker = cfg['marker']
    pose = Pose()
    pose.position.x = float(marker['x_m']) - 0.05
    pose.position.y = float(marker['spacing_m']) / 2.0
    pose.position.z = float(marker['center_height_m'])
    pose.orientation.w = 1.0
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    begin = time.monotonic()
    spawn('occluder', occluder_sdf(cfg), '', pose, 'world')
    latency = monitor.wait_zero(begin)
    if latency is None:
        return 'FAIL', 'no zero command within 5 s of covering the marker'
    all_zero, drift = monitor.observe_stopped()
    state, reason = monitor.state, monitor.error.reason if monitor.error else ''
    delete('occluder')
    resumed = monitor.wait_resumed()
    ok = all_zero and drift < 0.01 and state == 'SAFETY_STOP' and resumed
    return ('PASS' if ok else 'FAIL',
            'zero command %.2f s after covering; state=%s; estimator reason=%s; all zero for %.0f s=%s; '
            'drift=%.4f m; resumed after uncovering=%s'
            % (latency, state, reason, OBSERVE_S, all_zero, drift, resumed))


def check_railpause(monitor, cfg):
    return check_pause(monitor, cfg, 'RAIL_DRIVE')


def check_pause(monitor, cfg, target_state='APPROACH'):
    monitor.wait_moving(target_state)
    pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    begin = time.monotonic()
    pause()
    latency = monitor.wait_zero(begin)
    if latency is None:
        unpause()
        return 'FAIL', 'no zero command within 5 s of the image stream stopping'
    window_begin = time.monotonic()
    time.sleep(OBSERVE_S)
    window = [c for c in monitor.cmd_log if c[0] >= window_begin]
    all_zero = bool(window) and all(c[1] == 0.0 and c[2] == 0.0 for c in window)
    state = monitor.state
    unpause()
    resumed = monitor.wait_resumed()
    ok = all_zero and state == 'SAFETY_STOP' and resumed
    return ('PASS' if ok else 'FAIL',
            'zero command %.2f s (wall clock) after images stopped; state=%s; all zero for %.0f s=%s; '
            'resumed after unpause=%s' % (latency, state, OBSERVE_S, all_zero, resumed))


def check_nofeed(monitor, cfg):
    monitor.wait_moving()
    begin = time.monotonic()
    rosnode.kill_nodes(['/aruco_rail_estimator'])
    latency = monitor.wait_zero(begin)
    if latency is None:
        return 'FAIL', 'no zero command within 5 s of the estimator dying'
    all_zero, drift = monitor.observe_stopped()
    ok = all_zero and drift < 0.01 and monitor.state == 'SAFETY_STOP'
    return ('PASS' if ok else 'FAIL',
            'zero command %.2f s after estimator was killed; state=%s; all zero for %.0f s=%s; drift=%.4f m'
            % (latency, monitor.state, OBSERVE_S, all_zero, drift))


def check_blind(monitor, cfg):
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline and not monitor.error:
        time.sleep(0.05)
    if not monitor.error:
        return 'FAIL', 'no estimator output, so camera images were not flowing'
    time.sleep(2.0)
    start_pos = monitor.position()
    begin = time.monotonic()
    time.sleep(8.0)
    window = [c for c in monitor.cmd_log if c[0] >= begin]
    all_zero = bool(window) and all(c[1] == 0.0 and c[2] == 0.0 for c in window)
    end_pos = monitor.position()
    drift = math.hypot(end_pos[0] - start_pos[0], end_pos[1] - start_pos[1])
    ok = all_zero and drift < 0.01 and not monitor.error.valid
    return ('PASS' if ok else 'FAIL',
            'estimator reason=%s; %d commands in 8 s all zero=%s; drift=%.4f m'
            % (monitor.error.reason, len(window), all_zero, drift))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['occlude', 'pause', 'railpause', 'nofeed', 'blind'])
    parser.add_argument('--results-dir', default=os.path.join(os.getcwd(), 'results'))
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node('verify_safety')
    cfg = rospy.get_param('/rail_align')
    monitor = Monitor()
    verdict, detail = {'occlude': check_occlude, 'pause': check_pause, 'railpause': check_railpause,
                       'nofeed': check_nofeed, 'blind': check_blind}[args.mode](monitor, cfg)
    line = '%s | %s | %s' % (args.mode, verdict, detail)
    print('RESULT: ' + line)
    os.makedirs(args.results_dir, exist_ok=True)
    with open(os.path.join(args.results_dir, 'safety_check.txt'), 'a') as handle:
        handle.write(line + '\n')
    return 0 if verdict == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
