"""Generate a home-only Nav2 overlay; disabled features use the original file."""

import hashlib
import os
from pathlib import Path
import tempfile
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from launch.substitution import Substitution
from launch.substitutions import LaunchConfiguration
import yaml

from smart_delivery_core.optional_idle import home_nav_parameters, load_idle_config


class OptionalNavParameters(Substitution):
    """Resolve a precise home checker only after explicit feature configuration."""

    def perform(self, context):
        source = LaunchConfiguration('params_file').perform(context)
        config = load_idle_config(LaunchConfiguration('optional_idle_config').perform(context))
        if not (config['home']['enabled'] or config['people']['enabled']):
            return source
        map_path = LaunchConfiguration('map').perform(context)
        if hashlib.sha256(Path(map_path).read_bytes()).hexdigest() != config['map']['yaml_sha256']:
            raise ValueError('Optional poses are not verified for the selected map YAML')
        if not config['home']['enabled']:
            return source
        parameters = home_nav_parameters(yaml.safe_load(Path(source).read_text()), config)
        # Multiple goal checkers require an explicit normal checker. Do not
        # silently change a user-supplied normal tree to use the home tolerance.
        normal = ElementTree.parse(LaunchConfiguration('behavior_tree').perform(context))
        if any(node.get('goal_checker_id') != 'general_goal_checker'
               for node in normal.findall('.//FollowPath')):
            raise ValueError('Normal FollowPath BT must select general_goal_checker')
        bt_parameters = parameters['bt_navigator']['ros__parameters']
        through = bt_parameters.get('default_nav_through_poses_bt_xml') or str(
            Path(get_package_share_directory('nav2_bt_navigator')) / 'behavior_trees' /
            'navigate_through_poses_w_replanning_and_recovery.xml')
        tree = ElementTree.parse(through)
        for node in tree.findall('.//FollowPath'):
            node.set('goal_checker_id', 'general_goal_checker')
        with tempfile.NamedTemporaryFile(prefix='robot_normal_through_', suffix='.xml', delete=False) as file:
            tree.write(file, encoding='utf-8')
            bt_parameters['default_nav_through_poses_bt_xml'] = file.name
        descriptor, path = tempfile.mkstemp(prefix='robot_optional_home_', suffix='.yaml')
        with os.fdopen(descriptor, 'w') as file:
            yaml.safe_dump(parameters, file, allow_unicode=True)
        return path
