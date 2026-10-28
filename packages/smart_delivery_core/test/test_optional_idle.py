"""Dormant defaults, verified battery policy and non-unique directional scores."""

import copy
import hashlib
import json
import math
from pathlib import Path

from nav_msgs.msg import OccupancyGrid
import pytest
import yaml

from smart_delivery_core.optional_idle import (
    DirectionalPeople, HomeVoltageLatch, home_nav_parameters, load_idle_config,
    occupancy_fingerprint,
)

ROOT = Path(__file__).parents[1]
DEFAULT = ROOT / 'config/optional_idle_features.yaml'


def configuration():
    config = load_idle_config(DEFAULT)
    config['map'].update(coordinates_verified=True, yaml_sha256='a' * 64,
                         occupancy_fingerprint='b' * 64)
    config['home'].update(enabled=True, pose={'name': 'HOME', 'x': 1., 'y': 2., 'yaw': 0.},
                          battery_source_verified=True, low_voltage_v=10.0,
                          stop_claim_voltage_v=10.2, reset_voltage_v=10.7,
                          mode='precise_dock', xy_tolerance_m=.02, yaw_tolerance_rad=.035)
    config['people'].update(enabled=True,
        observation_pose={'name': 'OBS', 'x': 0., 'y': 0., 'yaw': 0.},
        standby_points=[{'name': 'E', 'x': 2., 'y': 0., 'yaw': 0.},
                        {'name': 'W', 'x': -2., 'y': 0., 'yaw': 0.},
                        {'name': 'E_FAR', 'x': 5., 'y': 0., 'yaw': 0.}])
    return config


def test_measured_people_enabled_but_unverified_main_battery_return_disabled():
    config = load_idle_config(DEFAULT)
    assert not config['home']['enabled'] and config['people']['enabled']
    assert not config['home']['battery_source_verified']
    assert config['home']['pose']['x'] == 0 and config['home']['low_voltage_v'] == 9.7
    assert config['home']['stop_claim_voltage_v'] == 10.0
    from smart_delivery_core.service_locations import STANDBY_POINTS
    assert config['people']['standby_points'] == STANDBY_POINTS


@pytest.mark.parametrize('section,key,value', [
    ('map', 'coordinates_verified', False), ('map', 'occupancy_fingerprint', ''),
    ('map', 'yaml_sha256', ''), ('home', 'pose', None),
    ('home', 'battery_source_verified', False), ('home', 'low_voltage_v', None),
    ('home', 'low_voltage_v', float('nan')), ('home', 'low_voltage_v', True),
    ('home', 'battery_topic', '/vehicle_battery_status'),
    ('home', 'battery_topic', '/power_status'),
    ('home', 'xy_tolerance_m', .2), ('people', 'observation_pose', None),
    ('people', 'standby_points', []), ('people', 'minimum_frames_per_bin', 1),
    ('people', 'minimum_heading_coverage', 0), ('people', 'bin_width_deg', 17),
    ('people', 'max_spin_resumes', 3), ('people', 'max_spin_resumes', True),
    ('people', 'safe_scan_samples', 2), ('people', 'max_consecutive_failures', 4),
    ('people', 'failure_cooldown_sec', 1000), ('people', 'observation_session_timeout_sec', 1),
])
def test_incomplete_enabled_feature_configuration_is_refused(tmp_path, section, key, value):
    config = configuration()
    config[section][key] = value
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(config), encoding='utf-8')
    with pytest.raises(ValueError):
        load_idle_config(path)


def test_complete_configuration_can_be_prepared_without_starting_any_node(tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(configuration()), encoding='utf-8')
    assert load_idle_config(path)['home']['enabled']


def sample(stamp, voltage=9.9, **extra):
    return dict(source_id='main_battery', sensor_ok=True, stamp_sec=stamp, voltage=voltage, **extra)


def test_home_requires_fifteen_continuous_fresh_low_voltage_seconds():
    latch = HomeVoltageLatch(configuration()['home'])
    for step in range(15):
        assert not latch.observe(sample(1000 + step), float(step), 1000 + step)
    assert latch.observe(sample(1015), 15.0, 1015)
    assert latch.latched
    latch.observe(sample(1016, 12.3), 16., 1016)
    assert latch.latched  # never auto-unlock because of voltage rebound


@pytest.mark.parametrize('payload', [
    {'source_id': 'charging_small_battery', 'stamp_sec': 1001, 'sensor_ok': True, 'voltage': 9.0},
    {'source_id': 'main_battery', 'stamp_sec': 990, 'sensor_ok': True, 'voltage': 9.0},
    {'source_id': 'main_battery', 'stamp_sec': 1001, 'sensor_ok': False, 'voltage': 9.0},
    {'source_id': 'main_battery', 'stamp_sec': 1001, 'sensor_ok': True, 'voltage': float('nan')},
    {'source_id': 'main_battery', 'stamp_sec': 1001, 'sensor_ok': True, 'voltage': 0.0},
    {}, [], None,
])
def test_wrong_source_invalid_or_stale_sample_cannot_trigger_home(payload):
    latch = HomeVoltageLatch(configuration()['home'])
    latch.observe(sample(1000), 0., 1000)
    assert not latch.observe(payload, 1., 1001)
    assert latch.low_since is None and not latch.healthy and not latch.latched


def test_battery_gap_or_high_sample_restarts_the_entire_low_interval():
    latch = HomeVoltageLatch(configuration()['home'])
    latch.observe(sample(1000), 0., 1000)
    latch.observe(sample(1014), 14., 1014)  # 14-second missing stream
    assert not latch.observe(sample(1015), 15., 1015)
    assert latch.low_since == 14.
    latch.observe(sample(1016, 12.), 16., 1016)
    assert latch.low_since is None
    latch.expire(20.)
    assert not latch.healthy


def test_repeated_battery_stamp_cannot_accumulate_low_duration():
    latch = HomeVoltageLatch(configuration()['home'])
    for step in range(20):
        latch.observe(sample(1000), float(step), 1000)
    assert not latch.latched


def test_occupancy_fingerprint_ignores_transport_time_but_binds_geometry_and_data():
    message = OccupancyGrid()
    message.info.width, message.info.height, message.info.resolution = 3, 1, .05
    message.info.origin.orientation.w = 1.
    message.data = [-1, 0, 100]
    original = occupancy_fingerprint(message)
    message.header.stamp.sec = 999
    assert occupancy_fingerprint(message) == original
    message.data[1] = 100
    assert occupancy_fingerprint(message) != original
    message.data[1] = 0
    message.info.origin.position.x = .1
    assert occupancy_fingerprint(message) != original


def test_home_checker_does_not_change_general_goal_tolerance_or_base_parameters():
    base = yaml.safe_load((ROOT / 'config/my_nav2_params.yaml').read_text(encoding='utf-8'))
    original = copy.deepcopy(base)
    result = home_nav_parameters(base, configuration())
    assert base == original
    old = original['controller_server']['ros__parameters']
    new = result['controller_server']['ros__parameters']
    assert new['general_goal_checker'] == old['general_goal_checker']
    assert new['home_goal_checker']['xy_goal_tolerance'] == .02
    assert new['home_goal_checker']['stateful'] is False
    assert home_nav_parameters(base, load_idle_config(DEFAULT)) == original


def test_disabled_navigation_substitution_returns_original_file_without_overlay():
    from launch import LaunchContext
    from smart_delivery_core.optional_nav_profile import OptionalNavParameters
    context = LaunchContext()
    config = load_idle_config(DEFAULT)
    config['people']['enabled'] = False
    import tempfile
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder)/'disabled.yaml'
        path.write_text(yaml.safe_dump(config))
        context.launch_configurations.update(params_file='original.yaml', optional_idle_config=str(path))
        assert OptionalNavParameters().perform(context) == 'original.yaml'


def test_disabled_delivery_does_not_construct_optional_runtime_or_subscriptions():
    import ast
    source = ROOT / 'smart_delivery_core/smart_delivery.py'
    main = next(node for node in ast.parse(source.read_text()).body
                if isinstance(node, ast.FunctionDef) and node.name == '_run_delivery')
    branch = next(node for node in main.body if isinstance(node, ast.If)
                  and ast.unparse(node.test).startswith("optional_config['home']['enabled']"))
    config = load_idle_config(DEFAULT)
    config['people']['enabled'] = False
    namespace = dict(optional_config=config, optional_runtime=None)
    # No navigator, callbacks or buffers are supplied: entering the branch would
    # fail, so this exercises the production activation condition directly.
    exec(compile(ast.Module(body=[branch], type_ignores=[]), str(source), 'exec'), namespace)
    assert namespace['optional_runtime'] is None


def test_optional_bt_ports_match_installed_jazzy_manifest():
    from xml.etree import ElementTree
    manifest = Path('/opt/ros/jazzy/share/nav2_behavior_tree/nav2_tree_nodes.xml')
    if not manifest.exists():
        pytest.skip('Requires the actual Jazzy behavior-tree manifest')
    descriptions = ElementTree.parse(manifest).getroot()
    for name in ('smart_home_navigate.xml', 'people_observe.xml', 'smart_delivery_navigate.xml'):
        for node in ElementTree.parse(ROOT / 'behavior_trees' / name).iter():
            if node.tag in {'Spin', 'FollowPath', 'Wait', 'ComputePathToPose'}:
                description = descriptions.find(f".//*[@ID='{node.tag}']")
                assert description is not None
                ports = {port.get('name') for port in description}
                assert set(node.attrib) - {'name'} <= ports


def test_enabled_home_profile_selects_distinct_normal_and_home_goal_checkers(tmp_path):
    from launch import LaunchContext
    from smart_delivery_core.optional_nav_profile import OptionalNavParameters
    config = configuration()
    map_file = tmp_path / 'test_map.yaml'
    map_file.write_text('image: unused.pgm\n')
    config['map']['yaml_sha256'] = hashlib.sha256(map_file.read_bytes()).hexdigest()
    config_file = tmp_path / 'features.yaml'
    config_file.write_text(yaml.safe_dump(config), encoding='utf-8')
    context = LaunchContext()
    context.launch_configurations.update(
        params_file=str(ROOT / 'config/my_nav2_params.yaml'),
        optional_idle_config=str(config_file), map=str(map_file),
        behavior_tree=str(ROOT / 'behavior_trees/smart_delivery_navigate.xml'))
    generated = Path(OptionalNavParameters().perform(context))
    values = yaml.safe_load(generated.read_text())
    assert 'home_goal_checker' in values['controller_server']['ros__parameters']['goal_checker_plugins']
    through = Path(values['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'])
    from xml.etree import ElementTree
    assert all(node.get('goal_checker_id') == 'general_goal_checker'
               for node in ElementTree.parse(through).findall('.//FollowPath'))
    generated.unlink()
    through.unlink()
    map_file.write_text('changed map\n')
    with pytest.raises(ValueError, match='selected map'):
        OptionalNavParameters().perform(context)


def scan_people(people, east_people=2):
    stamp = 0
    for heading_bin in range(24):
        heading = math.radians(heading_bin * 15 + 7.5)
        for _ in range(5):
            image_angle = math.degrees(math.atan2(math.sin(heading), math.cos(heading)))
            detections = ([{'class': 'Person', 'angle': image_angle, 'score': .9}] * east_people
                          if abs(image_angle) <= 34 else [])
            stamp += 1
            people.observe(detections, heading, stamp)


def test_people_directions_use_frame_median_not_sum_and_choose_nearest_safe_stop():
    config = configuration()['people']
    people = DirectionalPeople(config)
    scan_people(people)
    point = people.select(config['observation_pose'])
    assert point is not None and point['name'] == 'E'
    assert max(max(values, default=0) for values in people.frames) == 2


def test_no_people_or_partial_rotation_does_not_choose_an_arbitrary_destination():
    config = configuration()['people']
    people = DirectionalPeople(config)
    for stamp in range(30):
        people.observe([{'class': 'Person', 'angle': 0., 'score': .9}], 0., stamp)
    assert people.select(config['observation_pose']) is None
    people = DirectionalPeople(config)
    scan_people(people, east_people=0)
    assert people.select(config['observation_pose']) is None


def test_people_repeated_frames_low_confidence_and_non_people_are_not_counted():
    people = DirectionalPeople(configuration()['people'])
    values = [{'class': 'Glass', 'angle': 0., 'score': .99},
              {'class': 'Person', 'angle': 0., 'score': .1}]
    people.observe(values, 0., 10)
    people.observe([{'class': 'Person', 'angle': 0., 'score': .99}], 0., 10)
    assert max(max(items, default=0) for items in people.frames) == 0


def test_two_stage_battery_stop_at_ten_return_at_nine_point_seven():
    config=load_idle_config(DEFAULT)['home']
    latch=HomeVoltageLatch(config)
    for step in range(16):
        latch.observe(sample(1000+step,10.),float(step),1000+step)
    assert latch.admission_latched and not latch.latched
    for step in range(16,32):
        latch.observe(sample(1000+step,9.7),float(step),1000+step)
    assert latch.latched
    for step in range(32,48):
        latch.observe(sample(1000+step,12.),float(step),1000+step)
    assert latch.latched and latch.reset_ready(47.)


def test_admission_only_stop_recovers_with_hysteresis_but_return_latch_never_auto_unlocks():
    latch=HomeVoltageLatch(load_idle_config(DEFAULT)['home'])
    for step in range(16):
        latch.observe(sample(1000+step,9.9),float(step),1000+step)
    assert latch.admission_latched and not latch.latched
    for step in range(16,32):
        latch.observe(sample(1000+step,10.4),float(step),1000+step)
    assert latch.admission_latched
    for step in range(32,48):
        latch.observe(sample(1000+step,10.5),float(step),1000+step)
    assert not latch.admission_latched


def test_cutoff_latches_immediately_and_bad_voltage_cannot_fake_recovery():
    latch=HomeVoltageLatch(load_idle_config(DEFAULT)['home'])
    assert latch.observe(sample(1000,9.5),0.,1000)
    assert latch.cutoff and latch.admission_latched
    latch.observe(sample(1001,999.),1.,1001)
    assert not latch.healthy and not latch.reset_ready(100.)


@pytest.mark.parametrize('bearing,name', [(90.,'CYCU EE'),(180.,'座位燈'),(0.,'座位')])
def test_real_station_bearings_partition_each_direction_once(bearing,name):
    config=load_idle_config(DEFAULT)['people']
    people=DirectionalPeople(config)
    people.headings=set(range(people.count))
    people.frames=[[0]*5 for _ in range(people.count)]
    index=int(bearing/15)%people.count
    people.frames[index]=[2]*5
    assert people.select(config['observation_pose'])['name']==name
