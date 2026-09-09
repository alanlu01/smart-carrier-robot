"""Regression tests for the conservative NavigateToPose behavior tree."""

from pathlib import Path
from xml.etree import ElementTree


BT_PATH = (
    Path(__file__).parents[1]
    / 'behavior_trees'
    / 'smart_delivery_navigate.xml'
)


def test_navigation_behavior_tree_is_conservative():
    """Keep retries bounded and prohibit autonomous spin or reverse recovery."""
    root = ElementTree.parse(BT_PATH).getroot()
    main_recovery = root.find('./BehaviorTree/RecoveryNode')

    assert main_recovery is not None
    assert main_recovery.attrib['number_of_retries'] == '2'
    assert root.find('.//RateController').attrib['hz'] == '1.0'
    assert root.find('.//Wait').attrib['wait_duration'] == '2.0'
    assert root.findall('.//Spin') == []
    assert root.findall('.//BackUp') == []


def test_navigation_behavior_tree_only_has_contextual_costmap_clears():
    """Allow one planner and one controller contextual costmap clear only."""
    root = ElementTree.parse(BT_PATH).getroot()
    costmap_clears = root.findall('.//ClearEntireCostmap')

    assert [node.attrib['service_name'] for node in costmap_clears] == [
        'global_costmap/clear_entirely_global_costmap',
        'local_costmap/clear_entirely_local_costmap',
    ]
