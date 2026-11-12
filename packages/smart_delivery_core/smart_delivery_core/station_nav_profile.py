"""Station-only controller and native C++ velocity-selected collision polygons."""
from copy import deepcopy
import json
import math


def hull(points):
    points = sorted(set(points))
    def cross(o,a,b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for point in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],point)<=0: lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],point)<=0: upper.pop()
        upper.append(point)
    return lower[:-1]+upper[:-1]


def station_parameters(source):
    parameters = deepcopy(source)
    bt = parameters['bt_navigator']['ros__parameters']
    libraries = bt.setdefault('plugin_lib_names', [])
    if 'smart_navigation_policy_bt_node' not in libraries:
        libraries.append('smart_navigation_policy_bt_node')
    controller = parameters['controller_server']['ros__parameters']
    station = deepcopy(controller['FollowPath'])
    station.update(wz_max=.15,
                   vx_std=.10,vy_std=.10,wz_std=.05,batch_size=500,
                   ax_max=.3,ax_min=-.6,ay_max=.3,ay_min=-.6,az_max=.4)
    station['PreferForwardCritic']['enabled'] = False
    station['PathAngleCritic']['enabled'] = False
    station['GoalAngleCritic'].update(cost_weight=30.,threshold_to_consider=1.4)
    station['PathAlignCritic'].update(threshold_to_consider=.05,offset_from_furthest=1)
    station['PathFollowCritic']['offset_from_furthest'] = 1
    station['TwirlingCritic']['cost_weight'] = 10.
    station['ConstraintCritic']['cost_weight'] = 10.
    for axis in ('X','Y'):
        for departure in (False,True):
            profile = deepcopy(station)
            cap,cross = .15,.04
            main = math.sqrt(cap*cap-cross*cross)
            vx,vy = (main,cross) if axis=='X' else (cross,main)
            profile.update(vx_max=vx,vx_min=-vx,vy_max=vy,vy_min=-vy)
            name = 'Station'+('Exit' if departure else '')+axis
            if not departure:
                profile['plugin'] = 'smart_station_controller::StationMPPI'
                profile['station'] = dict(maximum_speed=.15,slow_speed=.06,
                                          slow_distance=.20,ramp_distance=.40)
            controller[name] = profile
            if name not in controller['controller_plugins']:
                controller['controller_plugins'].append(name)

    align = deepcopy(station)
    align.update(vx_max=.02,vx_min=-.02,vy_max=.02,vy_min=-.02,wz_max=.20,
                 vx_std=.02,vy_std=.02,wz_std=.08,az_max=.4,
                 plugin='nav2_rotation_shim_controller::RotationShimController',
                 primary_controller='nav2_mppi_controller::MPPIController',
                 rotate_to_goal_heading=True,rotate_to_heading_once=True,
                 rotate_to_heading_angular_vel=.20,max_angular_accel=.8,
                 angular_dist_threshold=.10,angular_disengage_threshold=.06,
                 simulate_ahead_time=1.,closed_loop=False)
    controller['StationAlign'] = align
    if 'StationAlign' not in controller['controller_plugins']:
        controller['controller_plugins'].append('StationAlign')
    controller['station_alignment_checker'] = dict(plugin='nav2_controller::SimpleGoalChecker',
        xy_goal_tolerance=.05,yaw_goal_tolerance=.06,stateful=False)
    if 'station_alignment_checker' not in controller['goal_checker_plugins']:
        controller['goal_checker_plugins'].append('station_alignment_checker')

    # Selecting one polygon is native CollisionMonitor work. Only its selected
    # outline is checked against scans; this is not 73 separate scan consumers.
    guard = parameters['collision_monitor']['ros__parameters']['NearFootprintStop']
    guard.update(type='velocity_polygon',holonomic=True)
    guard.pop('points',None)
    names = []
    circle = [(round(.32*math.cos(i*math.pi/8),5),round(.32*math.sin(i*math.pi/8),5)) for i in range(16)]
    def add(name,points,lo,hi,wlo,whi,start=-math.pi,end=math.pi):
        names.append(name)
        guard[name] = dict(points=json.dumps(points),linear_min=lo,linear_max=hi,
            theta_min=wlo,theta_max=whi,direction_start_angle=start,direction_end_angle=end)
    add('rotation_positive',circle,0.,.002,.001,10.)
    add('rotation_negative',circle,0.,.002,-10.,-.001)
    for band,(lo,hi) in enumerate(((0.,.08),(.08,.16),(.16,.36))):
        # The normal pipeline caps resultant speed at .336 m/s. Cover abnormal
        # higher commands conservatively too; never reuse the previous polygon.
        stopping = hi*.25+hi*hi/(2*.4)
        for sector in range(8):
            start,end = -math.pi+sector*math.pi/4,-math.pi+(sector+1)*math.pi/4
            for turn,(wlo,whi) in enumerate(((-10.,-.10),(.10,10.),(-.10,.10))):
                body = circle if turn<2 else [(-.25,-.19),(.25,-.19),(.25,.19),(-.25,.19)]
                # Inflate by sector midpoint too: endpoints alone cut a chord
                # through the possible stopping arc at the sector center.
                reach = stopping/math.cos(math.pi/8)
                shifted = [p for p in body]
                for a in (start,(start+end)/2,end):
                    shifted.extend((round(x+reach*math.cos(a),5),round(y+reach*math.sin(a),5)) for x,y in body)
                add(f'b{band}s{sector}t{turn}',hull(shifted),lo,hi,wlo,whi,start,end)
    add('fallback',hull([(x+dx,y+dy) for x,y in circle for dx in (-2.,2.) for dy in (-2.,2.)]),
        0.,100.,-100.,100.)
    guard['velocity_polygons'] = names
    return parameters
