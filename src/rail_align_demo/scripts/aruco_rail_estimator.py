#!/usr/bin/env python3
"""Camera image + camera_info -> rail-frame alignment error. Uses no simulator state."""
import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image

from rail_align_demo.geometry import RailPoseEstimator
from rail_align_demo.msg import AlignmentError


def pose_msg(rvec, tvec):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in tvec]
    angle = float(np.linalg.norm(rvec))
    if angle > 1e-9:
        axis = rvec / angle
        s = math.sin(angle / 2.0)
        pose.orientation.x, pose.orientation.y, pose.orientation.z = [float(v * s) for v in axis]
        pose.orientation.w = math.cos(angle / 2.0)
    else:
        pose.orientation.w = 1.0
    return pose


class EstimatorNode(object):
    def __init__(self):
        self.cfg = rospy.get_param('/rail_align')
        self.estimator = RailPoseEstimator(self.cfg)
        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.pub = rospy.Publisher('/rail_align/errors', AlignmentError, queue_size=5)
        self.debug_pub = rospy.Publisher('/rail_align/debug_image', Image, queue_size=1)
        rospy.Subscriber('/rail_cam/camera_info', CameraInfo, self.on_info, queue_size=1)
        # A large buffer keeps rospy from queueing stale frames behind the socket.
        rospy.Subscriber('/rail_cam/image_raw', Image, self.on_image, queue_size=1, buff_size=2 ** 25)

    def on_info(self, msg):
        self.K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.D, dtype=np.float64) if len(msg.D) else np.zeros(5)

    def on_image(self, msg):
        if self.K is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections, id_list = self.estimator.detect(gray)
        estimate = self.estimator.estimate(detections, id_list, self.K, self.D)

        out = AlignmentError()
        out.header = msg.header
        out.valid = estimate.valid
        out.reason = estimate.reason
        out.detected_ids = id_list
        out.reprojection_rms_px = estimate.reprojection_rms_px
        if estimate.left_marker_in_camera is not None:
            out.left_marker_in_camera = pose_msg(*estimate.left_marker_in_camera)
            out.right_marker_in_camera = pose_msg(*estimate.right_marker_in_camera)
        if estimate.valid:
            out.lateral_error_m = estimate.lateral_m
            out.heading_error_deg = math.degrees(estimate.heading_rad)
            out.longitudinal_error_m = estimate.longitudinal_m
        self.pub.publish(out)

        if self.debug_pub.get_num_connections() > 0:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(
                self.annotate(frame, detections, estimate, out), 'bgr8', msg.header))

    def annotate(self, frame, detections, estimate, out):
        axis = float(self.cfg['marker']['size_m']) / 2.0
        for pose in (estimate.left_marker_in_camera, estimate.right_marker_in_camera):
            if pose is not None:
                cv2.drawFrameAxes(frame, self.K, self.D, pose[0], pose[1], axis, 2)
        for marker_id, corners in detections.items():
            pts = corners.astype(np.int32)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.circle(frame, tuple(int(v) for v in pts[0]), 5, (0, 0, 255), -1)
            cv2.putText(frame, 'id %d' % marker_id, (int(pts[0][0]), int(pts[0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if out.valid:
            text = 'lat %+.3f m  head %+.2f deg  long %+.3f m  rms %.2f px' % (
                out.lateral_error_m, out.heading_error_deg,
                out.longitudinal_error_m, out.reprojection_rms_px)
            color = (0, 200, 0)
        else:
            text = 'INVALID: %s  ids=%s' % (out.reason, list(out.detected_ids))
            color = (0, 0, 255)
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 40), (255, 255, 255), -1)
        cv2.putText(frame, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        return frame


if __name__ == '__main__':
    rospy.init_node('aruco_rail_estimator')
    EstimatorNode()
    rospy.spin()
