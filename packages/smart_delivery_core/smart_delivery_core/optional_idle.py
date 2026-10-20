"""Pure policy for dormant home-return and directional crowd-aware parking."""

import copy
import hashlib
import json
import math
from pathlib import Path
import statistics

import yaml


def finite_number(value):
    """Reject boolean, NaN and infinity in navigation/battery settings."""
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def valid_pose(pose):
    """Require an explicitly named finite map-frame pose."""
    return (isinstance(pose, dict) and bool(str(pose.get('name', '')).strip())
            and all(finite_number(pose.get(key)) for key in ('x', 'y', 'yaw')))


def map_alignment_correction(previous, current, odom_x, odom_y):
    """Evaluate map-frame changes at the SAME odom point, not at map origins."""
    if not all(finite_number(v) for v in (*previous, *current, odom_x, odom_y)):
        raise ValueError('Alignment comparison requires finite planar transforms')
    def project(transform):
        x, y, angle = transform
        return (x+math.cos(angle)*odom_x-math.sin(angle)*odom_y,
                y+math.sin(angle)*odom_x+math.cos(angle)*odom_y)
    old, new = project(previous), project(current)
    return (math.hypot(new[0]-old[0], new[1]-old[1]),
            abs(math.atan2(math.sin(current[2]-previous[2]), math.cos(current[2]-previous[2]))))


def load_idle_config(path):
    """Fail closed on incomplete enabled features; disabled needs no poses."""
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('Optional idle config requires schema_version: 1')
    for section in ('map', 'home', 'people'):
        if not isinstance(data.get(section), dict):
            raise ValueError(f'Missing optional idle section: {section}')
    for section in ('home', 'people'):
        if not isinstance(data[section].get('enabled'), bool):
            raise ValueError(f'{section}.enabled must be a boolean')
    if not (data['home']['enabled'] or data['people']['enabled']):
        return data
    if data['map'].get('coordinates_verified') is not True:
        raise ValueError('Optional motion requires verified map coordinates')
    for key in ('yaml_sha256', 'occupancy_fingerprint'):
        digest = str(data['map'].get(key, ''))
        if len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
            raise ValueError(f'map.{key} must be a measured SHA256 digest')
    if data['home']['enabled']:
        home = data['home']
        if not valid_pose(home.get('pose')):
            raise ValueError('Home pose is not configured')
        if home.get('battery_source_verified') is not True:
            raise ValueError('The MAIN vehicle battery source must be verified')
        if home.get('battery_topic') in ('/power_status', '/vehicle_battery_status'):
            raise ValueError('Unverified legacy INA topic cannot trigger automatic home return')
        if not str(home.get('battery_topic', '')).startswith('/') or not home.get('battery_source_id'):
            raise ValueError('Dedicated verified battery topic/source is required')
        for key in ('low_voltage_v', 'stop_claim_voltage_v', 'reset_voltage_v',
                    'reset_duration_sec', 'cutoff_voltage_v', 'low_duration_sec', 'battery_timeout_sec',
                    'xy_tolerance_m', 'yaw_tolerance_rad', 'stopped_duration_sec',
                    'navigation_timeout_sec'):
            if not finite_number(home.get(key)) or home[key] <= 0:
                raise ValueError(f'home.{key} must be explicitly positive')
        if not home['cutoff_voltage_v'] < home['low_voltage_v'] < home['stop_claim_voltage_v'] < home['reset_voltage_v']:
            raise ValueError('Home voltage thresholds must be cutoff < return < stop-claim < reset')
        mode = home.get('mode', 'precise_dock')
        if mode not in ('return_point', 'precise_dock'):
            raise ValueError('Unsupported home mode')
        if home['xy_tolerance_m'] > (.10 if mode == 'return_point' else .05) or home['yaw_tolerance_rad'] > .10:
            raise ValueError('Home tolerances exceed the selected arrival mode')
    if data['people']['enabled']:
        people = data['people']
        if not valid_pose(people.get('observation_pose')):
            raise ValueError('People observation pose is not configured')
        points = people.get('standby_points')
        if not isinstance(points, list) or len(points) < 2 or not all(map(valid_pose, points)):
            raise ValueError('At least two measured people standby poses are required')
        if len({point['name'] for point in points}) != len(points):
            raise ValueError('People standby names must be unique')
        for key in ('idle_before_observation_sec', 'cooldown_sec', 'observation_timeout_sec',
                    'navigation_timeout_sec', 'scan_clearance_m', 'failure_cooldown_sec',
                    'stopped_before_spin_sec', 'spin_ready_wait_sec', 'observation_session_timeout_sec'):
            if not finite_number(people.get(key)) or people[key] <= 0:
                raise ValueError(f'people.{key} must be positive')
        for key, low, high in (('max_spin_resumes', 0, 2), ('max_consecutive_failures', 1, 3),
                               ('safe_scan_samples', 3, 10)):
            value = people.get(key)
            if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
                raise ValueError(f'people.{key} must be an integer in [{low},{high}]')
        if people['failure_cooldown_sec'] > people['cooldown_sec']:
            raise ValueError('Failure cooldown cannot exceed successful cooldown')
        if people['observation_session_timeout_sec'] < people['observation_timeout_sec']:
            raise ValueError('Session timeout cannot be shorter than one spin attempt')
        for key in ('minimum_confidence', 'minimum_heading_coverage'):
            if not finite_number(people.get(key)) or not 0 < people[key] <= 1:
                raise ValueError(f'people.{key} must be in (0,1]')
        fraction = people.get('minimum_positive_fraction', .15)
        if not finite_number(fraction) or not .10 <= fraction <= 1:
            raise ValueError('People positive-frame fraction must be in [0.10,1]')
        positive_frames = people.get('minimum_positive_frames', 3)
        if isinstance(positive_frames, bool) or not isinstance(positive_frames, int) or positive_frames < 3:
            raise ValueError('At least three independent positive frames are required')
        wait = people.get('localization_wait_sec', 20.)
        if not finite_number(wait) or not 5 <= wait <= 30:
            raise ValueError('People localization wait must be between 5 and 30 seconds')
        width = people.get('bin_width_deg')
        if not finite_number(width) or width < 5 or width > 30 or 360 % width:
            raise ValueError('Bin width must divide 360 and be between 5 and 30 degrees')
        frames = people.get('minimum_frames_per_bin')
        if not isinstance(frames, int) or isinstance(frames, bool) or frames < 3:
            raise ValueError('At least three fresh frames per bin are required')
        if not finite_number(people.get('camera_yaw_offset_rad')):
            raise ValueError('Camera yaw offset must be finite')
        if not finite_number(people.get('max_unknown_arc_deg')) or not 0 < people['max_unknown_arc_deg'] <= 15:
            raise ValueError('Unknown scan arc must be positive and at most 15 degrees')
    return data


def occupancy_fingerprint(message):
    """Bind coordinates to actual OccupancyGrid content, not just map frame."""
    info, origin = message.info, message.info.origin
    metadata = [info.width, info.height, round(info.resolution, 9),
                *[round(getattr(origin.position, key), 9) for key in ('x', 'y', 'z')],
                *[round(getattr(origin.orientation, key), 9) for key in ('x', 'y', 'z', 'w')]]
    digest = hashlib.sha256(json.dumps(metadata, separators=(',', ':')).encode())
    digest.update(bytes((int(value) & 0xff) for value in message.data))
    return digest.hexdigest()


class HomeVoltageLatch:
    """Require continuous fresh verified low voltage, then latch until reset."""

    def __init__(self, config, latched=False):
        self.config = config
        self.latched = latched
        self.low_since = None
        self.last_receipt = None
        self.last_stamp = None
        self.last_voltage = None
        self.healthy = False
        self.admission_latched = latched
        self.stop_claim_since = None
        self.recovered_since = None

    @property
    def cutoff(self):
        return self.last_voltage is not None and self.last_voltage <= self.config['cutoff_voltage_v']

    def reset_ready(self, now):
        return (self.healthy and not self.cutoff and self.recovered_since is not None
                and now-self.recovered_since >= self.config['reset_duration_sec'])

    def observe(self, payload, now, ros_now):
        """Ignore stale, repeated, wrong-source and unhealthy voltage samples."""
        try:
            stamp, voltage = payload['stamp_sec'], payload['voltage']
            valid = (payload.get('source_id') == self.config['battery_source_id']
                     and payload.get('sensor_ok') is True
                     and finite_number(stamp) and stamp > 0
                     and finite_number(voltage) and 6.0 <= voltage <= 13.5
                     and -0.1 <= ros_now - stamp <= self.config['battery_timeout_sec']
                     and (self.last_stamp is None or stamp > self.last_stamp))
        except (KeyError, TypeError):
            valid = False
        if not valid:
            self.low_since = None
            self.stop_claim_since = self.recovered_since = None
            self.healthy = False
            return False
        if (self.last_receipt is not None
                and now - self.last_receipt > self.config['battery_timeout_sec']):
            self.low_since = None
            self.stop_claim_since = self.recovered_since = None
        self.last_receipt, self.last_stamp = now, stamp
        self.last_voltage, self.healthy = voltage, True
        if voltage >= self.config['reset_voltage_v']:
            if self.recovered_since is None:
                self.recovered_since = now
            if not self.latched and self.reset_ready(now):
                self.admission_latched = False
        else:
            self.recovered_since = None
        if voltage <= self.config['stop_claim_voltage_v']:
            if self.stop_claim_since is None:
                self.stop_claim_since = now
            if now-self.stop_claim_since >= self.config['low_duration_sec']:
                self.admission_latched = True
        else:
            self.stop_claim_since = None
        if voltage > self.config['low_voltage_v']:
            self.low_since = None
        elif self.low_since is None:
            self.low_since = now
        elif now - self.low_since >= self.config['low_duration_sec']:
            self.latched = True
        if self.cutoff:
            self.latched = True  # Emergency never waits for the fifteen-second filter.
        if self.latched:
            self.admission_latched = True
        return self.latched

    def expire(self, now):
        """A missing stream cannot count toward the continuous low interval."""
        if self.last_receipt is None or now - self.last_receipt > self.config['battery_timeout_sec']:
            self.low_since = None
            self.stop_claim_since = self.recovered_since = None
            self.healthy = False


def home_pending_decision(task, *, home_requested, protected_id):
    """Physical handover is protected; never label an unstarted task completed."""
    resume = task.get('_resume_state')
    physical = resume in {'waiting_action', 'wrong_slot', 'verifying_action'}
    if home_requested and resume and not physical:
        return 'failed'
    if not resume and str(task.get('task_id')) != protected_id:
        return 'released'
    return None


class DirectionalPeople:
    """Score directions, not unique people; never sum detections across frames."""

    def __init__(self, config):
        self.config = config
        self.width = math.radians(config['bin_width_deg'])
        self.count = round(2 * math.pi / self.width)
        self.frames = [[] for _ in range(self.count)]
        self.headings = set()
        self.last_stamp = None
        self.received_frames = 0
        self.person_frames = 0
        self.qualified_detections = 0

    def observe(self, detections, heading, stamp):
        """Use median per-frame counts to reduce repeated sightings/noise."""
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.received_frames += 1
        heading += self.config['camera_yaw_offset_rad']
        self.headings.add(int((heading % (2 * math.pi)) / self.width) % self.count)
        counts = [0] * self.count
        for target in detections:
            if (not isinstance(target, dict) or target.get('class') != 'Person'
                    or not finite_number(target.get('score'))
                    or target['score'] < self.config['minimum_confidence']
                    or not finite_number(target.get('angle')) or abs(target['angle']) > 34):
                continue
            # The detector uses image-right positive; ROS yaw is left positive.
            bearing = heading - math.radians(target['angle'])
            counts[int((bearing % (2 * math.pi)) / self.width) % self.count] += 1
        self.person_frames += int(any(counts))
        self.qualified_detections += sum(counts)
        for index in range(self.count):
            middle = (index + 0.5) * self.width
            difference = math.atan2(math.sin(middle - heading), math.cos(middle - heading))
            if abs(difference) <= math.radians(34):
                self.frames[index].append(counts[index])
                self.frames[index] = self.frames[index][-60:]

    def summary(self, observation_pose):
        """Keep intermittent positives without accepting one-frame false demand."""
        positive = [sum(value > 0 for value in values) for values in self.frames]
        demand = []
        for values, count in zip(self.frames, positive):
            eligible = (len(values) >= self.config['minimum_frames_per_bin']
                        and count >= self.config.get('minimum_positive_frames', 3)
                        and count / len(values) >= self.config.get('minimum_positive_fraction', .15))
            if eligible:
                # Clip isolated large counts, not intermittent zeros. This is
                # an expected directional count, never a sum of sightings.
                cap = statistics.median(value for value in values if value > 0)
                demand.append(sum(min(value, cap) for value in values)/len(values))
            else:
                demand.append(0.)
        # Smooth only bins with qualified multi-frame evidence. The weights
        # sum to one, so a boundary sighting is not counted as three people.
        smoothed = [(.25*demand[(i-1) % self.count] + .5*demand[i]
                     + .25*demand[(i+1) % self.count]) for i in range(self.count)]
        directions = {p['name']: math.atan2(p['y']-observation_pose['y'],
                                           p['x']-observation_pose['x'])
                      for p in self.config['standby_points']}
        assigned = {p['name']: [] for p in self.config['standby_points']}
        distances = {p['name']: math.hypot(p['x']-observation_pose['x'],
                                          p['y']-observation_pose['y'])
                     for p in self.config['standby_points']}
        for index, value in enumerate(smoothed):
            bearing = (index+.5)*self.width
            name = min(directions, key=lambda name: (
                abs(math.atan2(math.sin(bearing-directions[name]), math.cos(bearing-directions[name]))),
                distances[name], name))
            assigned[name].append(value)
        candidates = []
        scores = {}
        for point in self.config['standby_points']:
            dx, dy = point['x'] - observation_pose['x'], point['y'] - observation_pose['y']
            values = assigned[point['name']]
            score = sum(values)/len(values) if values else 0.0
            scores[point['name']] = round(score, 5)
            candidates.append((-score, math.hypot(dx, dy), point['name'], point))
        best = min(candidates)
        coverage = len(self.headings)/self.count
        reason = ('insufficient_heading_coverage' if coverage < self.config['minimum_heading_coverage']
                  else 'no_person_detected' if not any(positive)
                  else 'insufficient_positive_evidence' if not any(demand)
                  else 'direction_selected')
        return {'reason': reason, 'received_frames': self.received_frames,
                'person_frames': self.person_frames, 'qualified_detections': self.qualified_detections,
                'heading_coverage': round(coverage, 3),
                'positive_frames_per_bin': positive, 'frames_per_bin': [len(v) for v in self.frames],
                'qualified_bins': sum(value > 0 for value in demand), 'sector_scores': scores,
                'selected': dict(best[3]) if reason == 'direction_selected' else None}

    def select(self, observation_pose):
        """Prefer the strongest supported direction, then the nearest safe stop."""
        return self.summary(observation_pose)['selected']


def home_nav_parameters(parameters, config):
    """Add a dedicated goal checker only when home return is enabled."""
    result = copy.deepcopy(parameters)
    if not config['home']['enabled']:
        return result
    controller = result['controller_server']['ros__parameters']
    plugins = list(controller['goal_checker_plugins'])
    if 'home_goal_checker' not in plugins:
        plugins.append('home_goal_checker')
    controller['goal_checker_plugins'] = plugins
    controller['home_goal_checker'] = {
        'plugin': 'nav2_controller::SimpleGoalChecker', 'stateful': False,
        'xy_goal_tolerance': config['home']['xy_tolerance_m'],
        'yaw_goal_tolerance': config['home']['yaw_tolerance_rad'],
    }
    return result
