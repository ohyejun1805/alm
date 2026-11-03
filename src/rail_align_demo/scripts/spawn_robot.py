#!/usr/bin/env python3
"""Spawn the cart at a named start pose from the config, with optional per-axis overrides."""
import math

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def main():
    rospy.init_node('spawn_robot')
    cfg = rospy.get_param('/rail_align')
    start = rospy.get_param('~start')
    poses = cfg['start_poses']
    if start not in poses and start != 'custom':
        raise rospy.ROSInitException('unknown start "%s"; choose from %s or custom' % (start, sorted(poses)))
    pose_cfg = dict(poses.get(start, {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0}))
    for key in ('x', 'y', 'yaw_deg'):
        override = str(rospy.get_param('~' + key, ''))
        if override != '':
            pose_cfg[key] = float(override)

    pose = Pose()
    pose.position.x = float(pose_cfg['x'])
    pose.position.y = float(pose_cfg['y'])
    pose.position.z = float(cfg['vehicle']['wheel_radius_m']) + 0.002
    yaw = math.radians(float(pose_cfg['yaw_deg']))
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)

    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    response = spawn('rail_cart', rospy.get_param('/robot_description'), '', pose, 'world')
    if not response.success:
        raise rospy.ROSException(response.status_message)
    rospy.loginfo('spawned rail_cart start=%s x=%.3f y=%.3f yaw=%.1fdeg',
                  start, pose_cfg['x'], pose_cfg['y'], pose_cfg['yaw_deg'])


if __name__ == '__main__':
    main()
