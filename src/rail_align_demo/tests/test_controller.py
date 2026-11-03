"""Closed-loop kinematic tests of the state machine. Run: python3 -m unittest discover -s tests"""
import copy
import math
import os
import sys
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
from rail_align_demo import controller as ctl  # noqa: E402

with open(os.path.join(HERE, '..', 'config', 'demo.yaml')) as handle:
    CFG = yaml.safe_load(handle)['rail_align']
STOP_X = -CFG['target']['standoff_m']
DT = 0.05


def measure(x, y, yaw):
    return y, yaw, STOP_X - x


def simulate(x, y, yaw_deg, limit_s=200.0):
    controller = ctl.AlignmentController(CFG)
    yaw = math.radians(yaw_deg)
    t = 0.0
    while t < limit_s and controller.state not in (ctl.ALIGNED, ctl.FAILED):
        v, w = controller.update(t, measure(x, y, yaw))
        x += v * math.cos(yaw) * DT
        y += v * math.sin(yaw) * DT
        yaw += w * DT
        t += DT
    return controller, x, y, yaw, t


class ControllerTest(unittest.TestCase):
    def test_all_start_poses_align(self):
        for name in ('left', 'right', 'left_inward', 'right_outward'):
            pose = CFG['start_poses'][name]
            controller, x, y, yaw, t = simulate(pose['x'], pose['y'], pose['yaw_deg'])
            self.assertEqual(controller.state, ctl.ALIGNED, name)
            self.assertTrue(controller.within_tolerance(y, yaw, STOP_X - x), name)

    def run_with_view(self, name):
        """Simulate one start pose; vision works only while both markers are inside the view."""
        half_fov = math.radians(CFG['camera']['hfov_deg']) / 2.0
        marker = CFG['marker']
        forward = CFG['camera']['forward_m']
        half = marker['size_m'] / 2.0
        edges = [s * marker['spacing_m'] / 2.0 + e for s in (1, -1) for e in (half, -half)]
        pose = CFG['start_poses'][name]
        controller = ctl.AlignmentController(CFG, pose.get('search_direction', 0))
        x, y, yaw, t = pose['x'], pose['y'], math.radians(pose['yaw_deg']), 0.0
        worst_after_found = 0.0
        found = False
        while t < 300.0 and controller.state not in (ctl.ALIGNED, ctl.FAILED):
            cam_x = x + forward * math.cos(yaw)
            cam_y = y + forward * math.sin(yaw)
            bearings = [abs(ctl._wrap(math.atan2(edge - cam_y, marker['x_m'] - cam_x) - yaw)) for edge in edges]
            visible = max(bearings) < half_fov
            found = found or visible
            if found:
                worst_after_found = max(worst_after_found, max(bearings))
            v, w = controller.update(t, measure(x, y, yaw) if visible else None)
            x += v * math.cos(yaw) * DT
            y += v * math.sin(yaw) * DT
            yaw += w * DT
            t += DT
        return controller, x, y, yaw, worst_after_found

    def test_markers_stay_in_view_for_all_start_poses(self):
        half_fov = math.radians(CFG['camera']['hfov_deg']) / 2.0
        for name in ('left', 'right', 'left_inward', 'right_outward'):
            controller, x, y, yaw, worst = self.run_with_view(name)
            self.assertEqual(controller.state, ctl.ALIGNED, name)
            self.assertLess(worst, half_fov - math.radians(2.0), name)

    def test_side_start_searches_then_aligns_without_retreating(self):
        for name in ('side_left', 'side_right'):
            controller, x, y, yaw, _ = self.run_with_view(name)
            self.assertEqual(controller.state, ctl.ALIGNED, name)
            self.assertEqual(controller.retreats, 0, name)
            self.assertTrue(controller.within_tolerance(y, yaw, STOP_X - x), name)

    def test_search_turns_only_with_images_and_gives_up(self):
        controller = ctl.AlignmentController(CFG, search_direction=-1)
        delay = CFG['control']['start_delay_s']
        self.assertEqual(controller.update(0.0, None, feed_ok=False), (0.0, 0.0))
        self.assertEqual(controller.update(1.0, None), (0.0, 0.0))  # settling delay
        v, w = controller.update(1.0 + delay, None)
        self.assertEqual((v, controller.state), (0.0, ctl.SEARCH))
        self.assertLess(w, 0.0)
        self.assertEqual(controller.update(1.1 + delay, None, feed_ok=False), (0.0, 0.0))
        controller.update(1.2 + delay + 2.0 * math.pi / CFG['search']['turn_rate'], None)
        self.assertEqual(controller.state, ctl.FAILED)
        self.assertEqual(controller.update(100.0, None), (0.0, 0.0))

    def test_no_search_without_a_direction(self):
        controller = ctl.AlignmentController(CFG)
        for t in (0.0, 5.0, 50.0):
            self.assertEqual(controller.update(t, None), (0.0, 0.0))
        self.assertEqual(controller.state, ctl.WAIT_VISION)

    def test_marker_loss_after_acquisition_stops_instead_of_searching(self):
        controller = ctl.AlignmentController(CFG, search_direction=1)
        controller.update(0.0, measure(-3.0, 0.2, 0.0))
        controller.update(CFG['control']['start_delay_s'], measure(-3.0, 0.2, 0.0))
        self.assertNotIn(controller.state, (ctl.WAIT_VISION, ctl.SEARCH))
        self.assertEqual(controller.update(10.0, None), (0.0, 0.0))
        self.assertEqual(controller.state, ctl.SAFETY_STOP)

    def test_no_lateral_velocity_needed_when_too_close(self):
        # Too near to converge on the first pass; must back up and retry rather than give up.
        controller, x, y, yaw, _ = simulate(STOP_X - 0.1, 0.15, 0.0)
        self.assertEqual(controller.state, ctl.ALIGNED)
        self.assertGreaterEqual(controller.retreats, 1)
        self.assertTrue(controller.within_tolerance(y, yaw, STOP_X - x))

    def test_waits_before_moving(self):
        controller = ctl.AlignmentController(CFG)
        self.assertEqual(controller.update(0.0, measure(-2.0, 0.1, 0.1)), (0.0, 0.0))
        self.assertEqual(controller.update(1.0, measure(-2.0, 0.1, 0.1)), (0.0, 0.0))

    def test_vision_loss_stops_and_resumes(self):
        controller = ctl.AlignmentController(CFG)
        controller.update(0.0, measure(-2.0, 0.0, 0.0))
        v, _ = controller.update(3.0, measure(-2.0, 0.0, 0.0))
        self.assertGreater(v, 0.0)
        before = controller.state
        self.assertEqual(controller.update(3.1, None), (0.0, 0.0))
        self.assertEqual(controller.state, ctl.SAFETY_STOP)
        v, _ = controller.update(3.2, measure(-2.0, 0.0, 0.0))
        self.assertEqual(controller.state, before)
        self.assertGreater(v, 0.0)

    def test_vision_loss_resets_hold_timer(self):
        controller = ctl.AlignmentController(CFG)
        controller.state = ctl.HOLD
        target = (0.0, 0.0, 0.0)
        controller.update(10.0, target)
        controller.update(10.9, None)
        controller.update(11.0, target)
        self.assertEqual(controller.state, ctl.HOLD)
        controller.update(11.9, target)
        self.assertEqual(controller.state, ctl.HOLD)
        controller.update(12.0, target)
        self.assertEqual(controller.state, ctl.ALIGNED)

    def test_aligned_is_latched_without_rail_drive(self):
        cfg = copy.deepcopy(CFG)
        cfg['rail_drive']['enabled'] = False
        controller = ctl.AlignmentController(cfg)
        controller.state = ctl.ALIGNED
        self.assertEqual(controller.update(1.0, (0.5, 0.5, 0.5)), (0.0, 0.0))
        self.assertEqual(controller.update(2.0, None), (0.0, 0.0))
        self.assertEqual(controller.state, ctl.ALIGNED)

    def rail_controller(self):
        """A controller that has just reached ALIGNED and paused for the configured time."""
        controller = ctl.AlignmentController(CFG)
        controller.state = ctl.ALIGNED
        self.assertEqual(controller.update(0.0, (0.0, 0.0, 0.0), True, (0.0, 0.0, 0.0)), (0.0, 0.0))
        self.assertEqual(controller.state, ctl.ALIGNED)
        controller.update(CFG['rail_drive']['pause_after_aligned_s'], (0.0, 0.0, 0.0), True, (0.0, 0.0, 0.0))
        self.assertEqual(controller.state, ctl.RAIL_DRIVE)
        return controller

    def test_rail_drive_ignores_marker_loss_and_reaches_end(self):
        controller = self.rail_controller()
        x, t = 0.0, 10.0
        while controller.state == ctl.RAIL_DRIVE and t < 200.0:
            v, w = controller.update(t, None, True, (x, 0.0, 0.0))
            self.assertLessEqual(v, CFG['rail_drive']['speed'])
            self.assertEqual(w, 0.0)
            x += v * DT
            t += DT
        self.assertEqual(controller.state, ctl.RAIL_END)
        self.assertAlmostEqual(x, CFG['rail_drive']['travel_m'], delta=0.02)
        self.assertEqual(controller.update(t, None, True, (x, 0.0, 0.0)), (0.0, 0.0))

    def test_rail_drive_stops_on_camera_or_odometry_loss_and_resumes(self):
        controller = self.rail_controller()
        controller.update(5.0, None, True, (0.5, 0.0, 0.0))
        self.assertEqual(controller.update(5.1, None, False, (0.5, 0.0, 0.0)), (0.0, 0.0))
        self.assertEqual(controller.state, ctl.SAFETY_STOP)
        self.assertEqual(controller.update(5.2, None, True, None), (0.0, 0.0))
        self.assertEqual(controller.state, ctl.SAFETY_STOP)
        v, _ = controller.update(5.3, None, True, (0.5, 0.0, 0.0))
        self.assertEqual(controller.state, ctl.RAIL_DRIVE)
        self.assertGreater(v, 0.0)

    def test_rail_drive_steers_out_the_last_camera_heading(self):
        controller = ctl.AlignmentController(CFG)
        controller.state = ctl.ALIGNED
        heading = math.radians(1.0)  # nose left of the rail
        controller.update(0.0, (0.0, heading, 0.0), True, (0.0, 0.0, 0.0))
        _, w = controller.update(CFG['rail_drive']['pause_after_aligned_s'], (0.0, heading, 0.0),
                                 True, (0.0, 0.0, 0.0))
        self.assertEqual(controller.state, ctl.RAIL_DRIVE)
        self.assertLess(w, 0.0)

    def test_outside_tolerance_never_aligns(self):
        controller = ctl.AlignmentController(CFG)
        controller.state = ctl.HOLD
        controller.retreats = CFG['control']['max_retreats']
        controller.update(0.0, (0.05, 0.0, 0.0))
        self.assertEqual(controller.state, ctl.FAILED)


if __name__ == '__main__':
    unittest.main()
