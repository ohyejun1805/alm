"""Alignment and rail-drive state machine; no ROS dependency.

Alignment uses only the camera-derived error. The rail section, where the markers are out
of view, uses wheel-encoder odometry from the last camera heading.
"""
import math

WAIT_VISION = 'WAIT_VISION'
SEARCH = 'SEARCH'
ALIGN_HEADING = 'ALIGN_HEADING'
APPROACH = 'APPROACH'
RETREAT = 'RETREAT'
FINAL_HEADING = 'FINAL_HEADING'
TRIM_LONGITUDINAL = 'TRIM_LONGITUDINAL'
HOLD = 'HOLD'
ALIGNED = 'ALIGNED'
RAIL_DRIVE = 'RAIL_DRIVE'
RAIL_END = 'RAIL_END'
FAILED = 'FAILED'
SAFETY_STOP = 'SAFETY_STOP'


def _clip(value, limit):
    return max(-limit, min(limit, value))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class AlignmentController(object):
    def __init__(self, cfg, search_direction=0):
        """search_direction: +1 CCW or -1 CW turn while no marker pair was ever seen; 0 never moves."""
        control = cfg['control']
        tol = cfg['tolerance']
        self.start_delay = float(control['start_delay_s'])
        self.v_max = float(control['v_max'])
        self.v_min = float(control['v_min'])
        self.w_max = float(control['w_max'])
        self.w_min = float(control['w_min'])
        self.k_long = float(control['k_longitudinal'])
        self.k_head = float(control['k_heading'])
        self.k_lat = float(control['k_lateral'])
        self.min_angle = math.radians(float(control['min_approach_angle_deg']))
        self.max_angle = math.radians(float(control['max_approach_angle_deg']))
        self.half_view = math.radians(float(cfg['camera']['hfov_deg'])) / 2.0 - math.radians(
            float(control['view_margin_deg']))
        self.camera_forward = float(cfg['camera']['forward_m'])
        marker = cfg['marker']
        self.marker_outer_y = float(marker['spacing_m']) / 2.0 + float(marker['size_m']) / 2.0
        # Distance from a vehicle at longitudinal error 0 to the marker plane.
        self.marker_depth = float(marker['x_m']) + float(cfg['target']['standoff_m'])
        self.align_tol = math.radians(float(control['align_tolerance_deg']))
        self.realign = math.radians(float(control['realign_threshold_deg']))
        self.arrive = float(control['arrive_longitudinal_m'])
        self.lat_inner = float(control['lateral_inner_m'])
        self.head_inner = math.radians(float(control['heading_inner_deg']))
        self.long_inner = float(control['longitudinal_inner_m'])
        self.retreat_speed = float(control['retreat_speed'])
        self.retreat_long = float(control['retreat_longitudinal_m'])
        self.max_retreats = int(control['max_retreats'])
        self.tol_lat = float(tol['lateral_m'])
        self.tol_head = math.radians(float(tol['heading_deg']))
        self.tol_long = float(tol['longitudinal_m'])
        self.hold_s = float(tol['hold_s'])
        rail = cfg['rail_drive']
        self.rail_enabled = bool(rail['enabled'])
        self.rail_pause = float(rail['pause_after_aligned_s'])
        self.rail_speed = float(rail['speed'])
        self.rail_accel = float(rail['acceleration'])
        self.rail_travel = float(rail['travel_m'])
        self.search_direction = int(search_direction)
        self.search_rate = float(cfg['search']['turn_rate'])
        self.search_limit_s = math.radians(float(cfg['search']['max_angle_deg'])) / self.search_rate
        self._search_boot = None
        self._search_since = None

        self.state = WAIT_VISION
        self.resume_state = WAIT_VISION
        self.retreats = 0
        self._first_valid = None
        self._hold_since = None
        self._last_heading = 0.0
        self._aligned_at = None
        self._rail_start = None
        self._rail_yaw = 0.0
        self._rail_v = 0.0
        self._rail_time = 0.0

    def within_tolerance(self, lateral, heading, longitudinal):
        return (abs(lateral) <= self.tol_lat and abs(heading) <= self.tol_head and
                abs(longitudinal) <= self.tol_long)

    def update(self, now, measurement, feed_ok=True, odom=None):
        """Advance one tick and return (v, w).

        measurement: (lateral m, heading rad, longitudinal m), or None when the marker pair
        is unusable. feed_ok: camera images are still arriving. odom: encoder (x, y, yaw), or
        None when stale. Only the rail drive reads the last two.
        """
        if measurement is not None:
            self._last_heading = measurement[1]
        if self.state in (RAIL_END, FAILED):
            return 0.0, 0.0
        if self.state == ALIGNED:
            return self._aligned(now, feed_ok, odom)
        if self.state == RAIL_DRIVE or (self.state == SAFETY_STOP and self.resume_state == RAIL_DRIVE):
            return self._rail(now, feed_ok, odom)
        if self.search_direction and self.state in (WAIT_VISION, SEARCH) and measurement is None:
            return self._search(now, feed_ok)
        if self.state == SEARCH:
            self.state = WAIT_VISION
        if measurement is None:
            self._hold_since = None
            self._first_valid = None
            if self.state == WAIT_VISION:
                return 0.0, 0.0
            if self.state != SAFETY_STOP:
                self.resume_state = self.state
                self.state = SAFETY_STOP
            return 0.0, 0.0
        if self.state == SAFETY_STOP:
            self.state = self.resume_state

        lateral, heading, longitudinal = measurement
        if self.state == WAIT_VISION:
            if self._first_valid is None:
                self._first_valid = now
            if now - self._first_valid < self.start_delay:
                return 0.0, 0.0
            self.state = ALIGN_HEADING

        handler = {
            ALIGN_HEADING: self._align_heading,
            APPROACH: self._approach,
            RETREAT: self._retreat,
            FINAL_HEADING: self._final_heading,
            TRIM_LONGITUDINAL: self._trim,
            HOLD: self._hold,
        }[self.state]
        return handler(now, lateral, heading, longitudinal)

    def _search(self, now, feed_ok):
        """Turn in place to find the pair. Only before first acquisition, bounded, needs images."""
        self._first_valid = None
        if not feed_ok:
            return 0.0, 0.0
        if self._search_boot is None:
            self._search_boot = now
        if now - self._search_boot < self.start_delay:
            return 0.0, 0.0
        if self._search_since is None:
            self._search_since = now
        if now - self._search_since > self.search_limit_s:
            self.state = FAILED
            return 0.0, 0.0
        self.state = SEARCH
        return 0.0, self.search_direction * self.search_rate

    def _view_limited_angle(self, lateral, longitudinal):
        """Largest approach angle that keeps the marker on the vehicle's own side in view."""
        depth = self.marker_depth + longitudinal
        angle = self.max_angle
        for _ in range(3):  # the camera position depends on the angle, which converges quickly
            camera_y = abs(lateral) - self.camera_forward * math.sin(angle)
            camera_depth = max(0.1, depth - self.camera_forward * math.cos(angle))
            angle = self.half_view - math.atan2(self.marker_outer_y - camera_y, camera_depth)
            angle = max(self.min_angle, min(self.max_angle, angle))
        return angle

    def _heading_reference(self, lateral, longitudinal):
        limit = self._view_limited_angle(lateral, longitudinal)
        return -_clip(math.atan(self.k_lat * lateral), limit)

    def _turn(self, delta):
        return _clip(self.k_head * delta, self.w_max)

    def _align_heading(self, now, lateral, heading, longitudinal):
        delta = _wrap(self._heading_reference(lateral, longitudinal) - heading)
        if abs(delta) <= self.align_tol:
            self.state = APPROACH
            return self._approach(now, lateral, heading, longitudinal)
        w = self._turn(delta)
        return 0.0, math.copysign(max(abs(w), self.w_min), w)

    def _approach(self, now, lateral, heading, longitudinal):
        if longitudinal <= self.arrive:
            if abs(lateral) <= self.lat_inner:
                self.state = FINAL_HEADING
            else:
                return self._begin_retreat()
            return 0.0, 0.0
        delta = _wrap(self._heading_reference(lateral, longitudinal) - heading)
        if abs(delta) > self.realign:
            self.state = ALIGN_HEADING
            return 0.0, 0.0
        v = max(self.v_min, min(self.v_max, self.k_long * longitudinal))
        return v * max(0.0, math.cos(delta)), self._turn(delta)

    def _begin_retreat(self):
        if self.retreats >= self.max_retreats:
            self.state = FAILED
            return 0.0, 0.0
        self.retreats += 1
        self.state = RETREAT
        return 0.0, 0.0

    def _retreat(self, now, lateral, heading, longitudinal):
        if longitudinal >= self.retreat_long:
            self.state = ALIGN_HEADING
            return 0.0, 0.0
        return -self.retreat_speed, self._turn(_wrap(-heading))

    def _final_heading(self, now, lateral, heading, longitudinal):
        if abs(heading) <= self.head_inner:
            self.state = HOLD
            return 0.0, 0.0
        w = self._turn(_wrap(-heading))
        return 0.0, math.copysign(max(abs(w), self.w_min), w)

    def _trim(self, now, lateral, heading, longitudinal):
        if abs(longitudinal) <= self.long_inner:
            self.state = FINAL_HEADING
            return 0.0, 0.0
        v = math.copysign(self.v_min, longitudinal)
        return v, self._turn(_wrap(-heading))

    def _hold(self, now, lateral, heading, longitudinal):
        if self.within_tolerance(lateral, heading, longitudinal):
            if self._hold_since is None:
                self._hold_since = now
            if now - self._hold_since >= self.hold_s:
                self.state = ALIGNED
            return 0.0, 0.0
        self._hold_since = None
        if abs(lateral) > self.tol_lat:
            return self._begin_retreat()
        if abs(heading) > self.tol_head:
            self.state = FINAL_HEADING
        else:
            self.state = TRIM_LONGITUDINAL
        return 0.0, 0.0

    def _aligned(self, now, feed_ok, odom):
        if self._aligned_at is None:
            self._aligned_at = now
        if (not self.rail_enabled or now - self._aligned_at < self.rail_pause or
                not feed_ok or odom is None):
            return 0.0, 0.0
        self._rail_start = odom[:2]
        # The rail direction in the odometry frame, from the last camera heading.
        self._rail_yaw = _wrap(odom[2] - self._last_heading)
        self._rail_v = 0.0
        self._rail_time = now
        self.state = RAIL_DRIVE
        return self._rail(now, feed_ok, odom)

    def _rail(self, now, feed_ok, odom):
        dt = max(0.0, now - self._rail_time)
        self._rail_time = now
        if not feed_ok or odom is None:
            self._rail_v = 0.0
            self.resume_state = RAIL_DRIVE
            self.state = SAFETY_STOP
            return 0.0, 0.0
        self.state = RAIL_DRIVE
        travelled = math.hypot(odom[0] - self._rail_start[0], odom[1] - self._rail_start[1])
        remaining = self.rail_travel - travelled
        if remaining <= 0.0:
            self.state = RAIL_END
            return 0.0, 0.0
        braking = max(self.v_min, self.k_long * remaining)
        self._rail_v = min(self.rail_speed, braking, self._rail_v + self.rail_accel * dt)
        return self._rail_v, self._turn(_wrap(self._rail_yaw - odom[2]))
