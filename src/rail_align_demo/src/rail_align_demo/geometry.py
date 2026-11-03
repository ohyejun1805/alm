"""ArUco detection and rail-frame pose estimation. No ROS dependency.

Rail frame (same as the Gazebo world frame): origin on the center line at the
entry line, +X into the rail, +Y left when facing into the rail, +Z up.
Marker faces point toward -X, so seen from the vehicle image-right is -Y.
"""
import math

import cv2
import numpy as np


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def rail_errors(x, y, yaw, target_x):
    """Return (lateral m, heading rad, longitudinal m) for a planar pose in the rail frame."""
    return y, wrap_pi(yaw), target_x - x


def target_x(cfg):
    return -float(cfg['target']['standoff_m'])


def _rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def camera_optical_in_base(cfg):
    """4x4 pose of the camera optical frame (x right, y down, z forward) in base_link.

    base_link sits at the axle center, wheel_radius above the ground.
    """
    cam = cfg['camera']
    pitch_up = math.radians(float(cam['pitch_up_deg']))
    link_to_optical = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    transform = np.eye(4)
    # A positive rotation about +Y tilts +X downward.
    transform[:3, :3] = _rot_y(-pitch_up).dot(link_to_optical)
    transform[:3, 3] = [float(cam['forward_m']), 0.0,
                        float(cam['height_m']) - float(cfg['vehicle']['wheel_radius_m'])]
    return transform


def marker_corners_in_rail(cfg, y_center):
    """Corners in ArUco order (TL, TR, BR, BL as seen from the vehicle), rail frame."""
    marker = cfg['marker']
    half = float(marker['size_m']) / 2.0
    x = float(marker['x_m'])
    z = float(marker['center_height_m'])
    return np.array([[x, y_center + half, z + half],
                     [x, y_center - half, z + half],
                     [x, y_center - half, z - half],
                     [x, y_center + half, z - half]])


def make_aruco_detector(dictionary_name):
    """Return (dictionary, detect(gray)); covers OpenCV 4.2 and >= 4.7 aruco APIs."""
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
    if hasattr(aruco, 'ArucoDetector'):
        params = aruco.DetectorParameters()
        params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        return dictionary, aruco.ArucoDetector(dictionary, params).detectMarkers
    params = aruco.DetectorParameters_create()
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    return dictionary, lambda gray: aruco.detectMarkers(gray, dictionary, parameters=params)


def render_marker(dictionary, marker_id, pixels):
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, pixels)
    return cv2.aruco.drawMarker(dictionary, marker_id, pixels)


class PoseEstimate(object):
    def __init__(self, valid=False, reason=''):
        self.valid = valid
        self.reason = reason
        self.lateral_m = 0.0
        self.heading_rad = 0.0
        self.longitudinal_m = 0.0
        self.reprojection_rms_px = 0.0
        self.base_in_rail = None
        self.left_marker_in_camera = None   # (rvec, tvec)
        self.right_marker_in_camera = None


class RailPoseEstimator(object):
    def __init__(self, cfg):
        self.cfg = cfg
        marker = cfg['marker']
        self.left_id = int(marker['left_id'])
        self.right_id = int(marker['right_id'])
        self.size = float(marker['size_m'])
        self.spacing = float(marker['spacing_m'])
        self.target_x = target_x(cfg)
        self.axle_height = float(cfg['vehicle']['wheel_radius_m'])
        self.gates = cfg['estimator']
        self.optical_in_base = camera_optical_in_base(cfg)
        self.base_in_optical = np.linalg.inv(self.optical_in_base)
        self.dictionary, self._detect = make_aruco_detector(marker['dictionary'])
        half = self.size / 2.0
        # Required corner layout of SOLVEPNP_IPPE_SQUARE, marker frame.
        self.square = np.array([[-half, half, 0.0], [half, half, 0.0],
                                [half, -half, 0.0], [-half, -half, 0.0]])
        self.object_points = np.vstack([
            marker_corners_in_rail(cfg, self.spacing / 2.0),
            marker_corners_in_rail(cfg, -self.spacing / 2.0)])

    def detect(self, gray):
        """Return ({id: 4x2 corners}, [all detected ids])."""
        corners, ids, _ = self._detect(gray)
        if ids is None:
            return {}, []
        id_list = [int(i) for i in np.asarray(ids).flatten()]
        return {i: np.asarray(c, dtype=np.float64).reshape(4, 2)
                for i, c in zip(id_list, corners)}, id_list

    def estimate(self, detections, id_list, K, D):
        """Fuse the marker pair into a vehicle pose in the rail frame."""
        for marker_id in (self.left_id, self.right_id):
            if marker_id not in detections:
                return PoseEstimate(reason='missing_marker_%d' % marker_id)
            if id_list.count(marker_id) > 1:
                return PoseEstimate(reason='duplicate_marker_%d' % marker_id)

        result = PoseEstimate(valid=True)
        result.left_marker_in_camera = self._single_marker(detections[self.left_id], K, D)
        result.right_marker_in_camera = self._single_marker(detections[self.right_id], K, D)
        measured_spacing = np.linalg.norm(
            result.left_marker_in_camera[1] - result.right_marker_in_camera[1])
        if abs(measured_spacing - self.spacing) > float(self.gates['max_spacing_error_m']):
            result.valid = False
            result.reason = 'spacing_mismatch_%.3f' % measured_spacing
            return result

        farthest = max(np.linalg.norm(result.left_marker_in_camera[1]),
                       np.linalg.norm(result.right_marker_in_camera[1]))
        if farthest > float(self.gates['max_marker_distance_m']):
            result.valid = False
            result.reason = 'too_far_%.2fm' % farthest
            return result

        image_points = np.vstack([detections[self.left_id], detections[self.right_id]])
        best = self._joint_pose(image_points, K, D)
        if best is None:
            result.valid = False
            result.reason = 'pnp_failed'
            return result
        base_in_rail, rms, height_error, tilt_deg = best
        result.reprojection_rms_px = rms
        if rms > float(self.gates['max_reprojection_px']):
            result.valid = False
            result.reason = 'reprojection_%.2fpx' % rms
            return result
        if height_error > float(self.gates['max_height_error_m']):
            result.valid = False
            result.reason = 'height_%.3fm' % height_error
            return result
        if tilt_deg > float(self.gates['max_tilt_deg']):
            result.valid = False
            result.reason = 'tilt_%.1fdeg' % tilt_deg
            return result

        yaw = math.atan2(base_in_rail[1, 0], base_in_rail[0, 0])
        result.base_in_rail = base_in_rail
        result.lateral_m, result.heading_rad, result.longitudinal_m = rail_errors(
            base_in_rail[0, 3], base_in_rail[1, 3], yaw, self.target_x)
        return result

    def _single_marker(self, corners, K, D):
        _, rvec, tvec = cv2.solvePnP(self.square, corners, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        return rvec.flatten(), tvec.flatten()

    def _joint_pose(self, image_points, K, D):
        """Solve all eight coplanar corners and resolve the planar two-fold ambiguity.

        Both IPPE candidates are refined; the one implying a vehicle that sits on
        the ground with the configured axle height wins over a marginally lower
        reprojection error.
        """
        # OpenCV 4.2 rejects its own default for reprojectionError, so one is always passed.
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            self.object_points, image_points, K, D, flags=cv2.SOLVEPNP_IPPE,
            reprojectionError=np.zeros((2, 1), dtype=np.float64))
        best = None
        best_score = float('inf')
        for i in range(count):
            rvec, tvec = cv2.solvePnPRefineLM(
                self.object_points, image_points, K, D, rvecs[i].copy(), tvecs[i].copy())
            projected, _ = cv2.projectPoints(self.object_points, rvec, tvec, K, D)
            residual = projected.reshape(-1, 2) - image_points
            rms = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))

            rail_in_optical = np.eye(4)
            rail_in_optical[:3, :3] = cv2.Rodrigues(rvec)[0]
            rail_in_optical[:3, 3] = tvec.flatten()
            base_in_rail = np.linalg.inv(rail_in_optical).dot(self.base_in_optical)
            height_error = abs(base_in_rail[2, 3] - self.axle_height)
            tilt_deg = math.degrees(math.acos(min(1.0, max(-1.0, base_in_rail[2, 2]))))
            score = rms + 10.0 * height_error + 0.2 * tilt_deg
            if score < best_score:
                best_score = score
                best = (base_in_rail, rms, height_error, tilt_deg)
        return best
