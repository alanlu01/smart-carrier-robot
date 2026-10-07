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
        for key in ('low_voltage_v', 'low_duration_sec', 'battery_timeout_sec',
                    'xy_tolerance_m', 'yaw_tolerance_rad', 'stopped_duration_sec',
                    'navigation_timeout_sec'):
            if not finite_number(home.get(key)) or home[key] <= 0:
                raise ValueError(f'home.{key} must be explicitly positive')
        if home['xy_tolerance_m'] > 0.05 or home['yaw_tolerance_rad'] > 0.10:
            raise ValueError('Home tolerances must stay tighter than normal parking')
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
                    'navigation_timeout_sec', 'scan_clearance_m'):
            if not finite_number(people.get(key)) or people[key] <= 0:
                raise ValueError(f'people.{key} must be positive')
        for key in ('minimum_confidence', 'minimum_heading_coverage'):
            if not finite_number(people.get(key)) or not 0 < people[key] <= 1:
                raise ValueError(f'people.{key} must be in (0,1]')
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

    def observe(self, payload, now, ros_now):
        """Ignore stale, repeated, wrong-source and unhealthy voltage samples."""
        try:
            stamp, voltage = payload['stamp_sec'], payload['voltage']
            valid = (payload.get('source_id') == self.config['battery_source_id']
                     and payload.get('sensor_ok') is True
                     and finite_number(stamp) and stamp > 0
                     and finite_number(voltage) and voltage > 0
                     and -0.1 <= ros_now - stamp <= self.config['battery_timeout_sec']
                     and (self.last_stamp is None or stamp > self.last_stamp))
        except (KeyError, TypeError):
            valid = False
        if not valid:
            self.low_since = None
            self.healthy = False
            return False
        if (self.last_receipt is not None
                and now - self.last_receipt > self.config['battery_timeout_sec']):
            self.low_since = None
        self.last_receipt, self.last_stamp = now, stamp
        self.last_voltage, self.healthy = voltage, True
        if voltage >= self.config['low_voltage_v']:
            self.low_since = None
        elif self.low_since is None:
            self.low_since = now
        elif now - self.low_since >= self.config['low_duration_sec']:
            self.latched = True
        return self.latched

    def expire(self, now):
        """A missing stream cannot count toward the continuous low interval."""
        if self.last_receipt is None or now - self.last_receipt > self.config['battery_timeout_sec']:
            self.low_since = None
            self.healthy = False


class DirectionalPeople:
    """Score directions, not unique people; never sum detections across frames."""

    def __init__(self, config):
        self.config = config
        self.width = math.radians(config['bin_width_deg'])
        self.count = round(2 * math.pi / self.width)
        self.frames = [[] for _ in range(self.count)]
        self.headings = set()
        self.last_stamp = None

    def observe(self, detections, heading, stamp):
        """Use median per-frame counts to reduce repeated sightings/noise."""
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
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
        for index in range(self.count):
            middle = (index + 0.5) * self.width
            difference = math.atan2(math.sin(middle - heading), math.cos(middle - heading))
            if abs(difference) <= math.radians(34):
                self.frames[index].append(counts[index])
                self.frames[index] = self.frames[index][-60:]

    def select(self, observation_pose):
        """Prefer the strongest measured direction, then the nearest safe stop."""
        if len(self.headings) / self.count < self.config['minimum_heading_coverage']:
            return None
        demand = [statistics.median(values) if len(values) >= self.config['minimum_frames_per_bin']
                  else 0.0 for values in self.frames]
        candidates = []
        for point in self.config['standby_points']:
            dx, dy = point['x'] - observation_pose['x'], point['y'] - observation_pose['y']
            bearing = math.atan2(dy, dx)
            score = max((value for index, value in enumerate(demand)
                         if abs(math.atan2(math.sin((index + 0.5) * self.width - bearing),
                                           math.cos((index + 0.5) * self.width - bearing)))
                         <= math.pi / 4), default=0.0)
            candidates.append((-score, math.hypot(dx, dy), point['name'], point))
        best = min(candidates)
        return dict(best[3]) if best[0] <= -1.0 else None


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
