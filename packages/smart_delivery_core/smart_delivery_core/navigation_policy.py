"""Bind a native BT/controller selection to one exact stamped map goal."""
import json
from pathlib import Path


PROFILES = {
    'station_x.xml': ('StationX', 'general_goal_checker'),
    'station_y.xml': ('StationY', 'general_goal_checker'),
    'station_exit_x.xml': ('StationExitX', 'general_goal_checker'),
    'station_exit_y.xml': ('StationExitY', 'general_goal_checker'),
    'station_align.xml': ('StationAlign', 'station_alignment_checker'),
}


def navigation_policy(pose, tree, share):
    """Return a policy only for package-owned station and ordinary trees."""
    share = Path(share).resolve()
    path = (Path(tree) if tree else share/'behavior_trees/smart_delivery_navigate.xml').resolve()
    owned = {(share/'behavior_trees'/name).resolve():name
             for name in ('smart_delivery_navigate.xml',*PROFILES)}
    name = owned.get(path)
    if name is None:
        return None  # Match exact files, including colcon's per-file symlinks.
    controller, checker = PROFILES.get(name, ('FollowPath', 'general_goal_checker'))
    p, q, stamp = pose.pose.position, pose.pose.orientation, pose.header.stamp
    payload = dict(schema_version=1, frame=pose.header.frame_id,
        stamp_ns=stamp.sec*10**9+stamp.nanosec,
        position=[p.x,p.y,p.z], orientation=[q.x,q.y,q.z,q.w],
        controller=controller, goal_checker=checker, station=name in PROFILES)
    return str(share/'behavior_trees/smart_owned_navigate.xml'), json.dumps(payload)
