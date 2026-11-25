"""Counterfactual policy checks on extracted bag data; no ROS replay or motion."""
import argparse
import bisect
import collections
import datetime as dt
import json
from pathlib import Path
import subprocess
import sys
import types

ROOT=Path(__file__).resolve().parents[1]
for package in ('smart_delivery_core','power_monitor'):
    sys.path.insert(0,str(ROOT/'packages'/package))
from smart_delivery_core.standby_idle_clock import StandbyIdleClock
from power_monitor.power_status import SlotStateTracker


def baseline(commit, package, module):
    path=f'packages/{package}/{package}/{module}.py'
    source=subprocess.check_output(['git','show',f'{commit}:{path}'],cwd=ROOT,text=True)
    loaded=types.ModuleType(f'baseline_{module}')
    sys.modules[loaded.__name__]=loaded
    exec(compile(source,path,'exec'),loaded.__dict__)
    return loaded


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('extracted_json',type=Path)
    parser.add_argument('--baseline',default='3ab7bfababa1303a61019da2355cedb5958364e7')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    rows=json.loads(args.extracted_json.read_text('utf-8-sig'))['rows']
    old_idle=baseline(args.baseline,'smart_delivery_core','standby_idle_clock').StandbyIdleClock
    old_power=baseline(args.baseline,'power_monitor','power_status').SlotStateTracker
    clocks={'baseline':old_idle(),'candidate':StandbyIdleClock()}
    zone=dt.timezone(dt.timedelta(hours=8))
    def label(t):return dt.datetime.fromtimestamp(t,zone).strftime('%H:%M:%S.%f')[:-3]
    odoms=rows['/chassis/odom_local'];odom_stamps=[t for t,_ in odoms]
    first_due={};maximum_motion=0.;anchor=None;last=None
    for t,p in rows['/smart_carrier/optional_idle_state']:
        if not '15:15:03.9'<=label(t)<='15:16:53.5':continue
        parked=p['idle_clock']['map_reference'];assert parked and p['state']=='idle'
        safety=p['safety_check'];pose=p['idle_clock']['observed_pose']
        op=odoms[max(0,bisect.bisect_right(odom_stamps,t)-1)][1]
        physical=tuple(op['pose']);velocity=op['velocity']
        if anchor is None:anchor=physical
        maximum_motion=max(maximum_motion,((physical[0]-anchor[0])**2+(physical[1]-anchor[1])**2)**.5)
        stationary=max(abs(v) for v in velocity[:2])<=.02 and abs(velocity[2])<=.03
        pose_valid=safety['tf']['valid']
        transient=(not safety['safe'] and safety['failed_checks']==['map_pose_valid']
                   and safety['localization_phase']=='LOCALIZED' and safety['checks']['odom_fresh'])
        for name,c in clocks.items():
            if c.point is None:c.mark_parked(parked)
            c.update(t,parked=True,safe=safety['safe'],stationary=stationary,
                pose=dict(zip(('x','y','yaw'),pose)) if pose_valid else None,
                odom=physical if safety['checks']['odom_fresh'] else None,transient_tf_gap=transient)
            if c.elapsed>=60. and name not in first_due:first_due[name]=label(t)
        last=t
    assert maximum_motion<.001 and clocks['candidate'].elapsed>clocks['baseline'].elapsed+40.
    assert 'candidate' in first_due
    trackers={name:[kind(confirm_samples=3,weak_presence_min_a=.010) for _ in range(3)]
              for name,kind in (('baseline',old_power),('candidate',SlotStateTracker))}
    counts={name:collections.Counter() for name in trackers};transitions=[];previous={}
    removals={name:{} for name in trackers}
    for t,p in rows['/power_status']:
        for name,items in trackers.items():
            for slot,tracker in enumerate(items,1):
                ch=p[f'ch{slot}'];supply=p['slot_supply']
                valid=ch['sensor_ok'] and (supply['healthy'] or supply.get('recovery_mode')=='communication')
                state=tracker.update(ch['current'],ch['voltage'],sensor_ok=valid,now=t)
                for start,end in (('15:14:23','15:14:30'),('15:32:49','15:32:55')):
                    if slot==1 and start<=label(t)<=end and state=='empty' and start not in removals[name]:
                        removals[name][start]=label(t)
                if slot==2 and '15:24:48'<=label(t)<='15:25:00':counts[name][state]+=1
                if slot==2 and '15:24:30'<=label(t)<='15:25:00' and previous.get(name)!=state:
                    transitions.append(dict(at=label(t),implementation=name,state=state))
                if slot==2:previous[name]=state
    assert counts['baseline']['empty']>0 and counts['candidate']['empty']==0,counts
    assert removals['baseline']==removals['candidate'] and len(removals['candidate'])==2,removals
    result=dict(passed=True,offline=True,hardware_commands=0,
        limitations='Counterfactual state decisions only; not proof of future physical docking or CPU savings.',
        idle=dict(first_60_seconds=first_due,maximum_odom_motion_m=maximum_motion,
            final={name:c.snapshot(last) for name,c in clocks.items()}),
        confirmed_removal_times=removals,
        user_confirmed_occupied_ch2=dict(window=['15:24:48','15:25:00'],counts=counts,transitions=transitions))
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(passed=True,first_idle_due=first_due,ch2_counts=counts),ensure_ascii=False))


if __name__=='__main__':main()
