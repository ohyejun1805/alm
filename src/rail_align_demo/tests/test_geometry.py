"""Synthetic projection tests. Run: python3 -m unittest discover -s tests"""
import math
import os
import sys
import unittest

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
from rail_align_demo.geometry import (RailPoseEstimator, camera_optical_in_base,  # noqa: E402
                                      marker_corners_in_rail)

with open(os.path.join(HERE, '..', 'config', 'demo.yaml')) as handle:
    CFG = yaml.safe_load(handle)['rail_align']


def camera_matrix():
    cam = CFG['camera']
    focal = cam['width_px'] / (2.0 * math.tan(math.radians(cam['hfov_deg']) / 2.0))
    return np.array([[focal, 0.0, cam['width_px'] / 2.0],
                     [0.0, focal, cam['height_px'] / 2.0],
                     [0.0, 0.0, 1.0]])


def project(x, y, yaw_deg, noise_px=0.0, seed=0):
    yaw = math.radians(yaw_deg)
    base_in_rail = np.eye(4)
    base_in_rail[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    base_in_rail[:3, 3] = [x, y, CFG['vehicle']['wheel_radius_m']]
    rail_in_optical = np.linalg.inv(base_in_rail.dot(camera_optical_in_base(CFG)))
    half = CFG['marker']['spacing_m'] / 2.0
    rng = np.random.RandomState(seed)
    out = {}
    for marker_id, y_center in ((CFG['marker']['left_id'], half), (CFG['marker']['right_id'], -half)):
        rvec, _ = cv2.Rodrigues(rail_in_optical[:3, :3])
        pts, _ = cv2.projectPoints(marker_corners_in_rail(CFG, y_center), rvec,
                                   rail_in_optical[:3, 3], camera_matrix(), np.zeros(5))
        out[marker_id] = pts.reshape(4, 2) + rng.normal(0.0, noise_px, (4, 2)) if noise_px else pts.reshape(4, 2)
    return out


class GeometryTest(unittest.TestCase):
    def setUp(self):
        self.est = RailPoseEstimator(CFG)

    def solve(self, x, y, yaw_deg, **kwargs):
        detections = project(x, y, yaw_deg, **kwargs)
        return self.est.estimate(detections, list(detections), camera_matrix(), np.zeros(5))

    def test_optical_frame_looks_forward(self):
        optical = camera_optical_in_base(CFG)
        forward = optical[:3, 2]
        self.assertGreater(forward[0], 0.9)
        self.assertGreater(forward[2], 0.0)  # pitched up
        self.assertAlmostEqual(optical[1, 0], -1.0, places=6)  # image right is vehicle -Y

    def test_values_and_signs_without_noise(self):
        for x, y, yaw in [(-3.0, 0.3, 12.0), (-3.0, -0.25, 15.0), (-1.4, 0.0, 0.0), (-2.0, 0.1, -10.0)]:
            result = self.solve(x, y, yaw)
            self.assertTrue(result.valid, result.reason)
            self.assertAlmostEqual(result.lateral_m, y, places=4)
            self.assertAlmostEqual(math.degrees(result.heading_rad), yaw, places=3)
            self.assertAlmostEqual(result.longitudinal_m, -CFG['target']['standoff_m'] - x, places=4)

    def test_mirror_ambiguity_with_pixel_noise(self):
        for seed in range(30):
            result = self.solve(-3.0, 0.3, 12.0, noise_px=0.2, seed=seed)
            self.assertTrue(result.valid, result.reason)
            self.assertLess(abs(math.degrees(result.heading_rad) - 12.0), 3.0)
            self.assertLess(abs(result.lateral_m - 0.3), 0.05)

    def test_wheels_straddle_rails_and_clear_markers(self):
        vehicle, rail, marker = CFG['vehicle'], CFG['rail'], CFG['marker']
        wheel_inner = (vehicle['wheel_separation_m'] - vehicle['wheel_width_m']) / 2.0
        wheel_outer = (vehicle['wheel_separation_m'] + vehicle['wheel_width_m']) / 2.0
        rail_outer = (rail['gauge_m'] + rail['width_m']) / 2.0
        self.assertGreater(wheel_inner, rail_outer)
        self.assertLess(wheel_inner - rail_outer, 0.06)  # "just outside" the rail
        self.assertLess(vehicle['body_width_m'] / 2.0, wheel_inner)
        plate_half = marker['size_m'] * 4.0 / 3.0 / 2.0
        self.assertGreater(marker['spacing_m'] / 2.0 - plate_half, wheel_outer)

    def test_rejects_markers_beyond_the_reliable_range(self):
        result = self.solve(-7.0, 0.0, 0.0)
        self.assertFalse(result.valid)
        self.assertIn('too_far', result.reason)

    def test_requires_both_markers(self):
        detections = project(-2.0, 0.0, 0.0)
        del detections[CFG['marker']['right_id']]
        result = self.est.estimate(detections, list(detections), camera_matrix(), np.zeros(5))
        self.assertFalse(result.valid)
        self.assertIn('missing_marker', result.reason)

    def test_swapped_ids_rejected(self):
        detections = project(-2.0, 0.0, 0.0)
        left, right = CFG['marker']['left_id'], CFG['marker']['right_id']
        detections[left], detections[right] = detections[right], detections[left]
        result = self.est.estimate(detections, list(detections), camera_matrix(), np.zeros(5))
        self.assertFalse(result.valid)


if __name__ == '__main__':
    unittest.main()
