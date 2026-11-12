"""Compare Dijkstra/A* offline, never start a ROS node or replay a recording."""
import argparse
import datetime
import json
import math
from pathlib import Path
import struct
import subprocess

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import distance_transform_edt
from smart_delivery_core.station_geometry import StationGrid


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('bag');parser.add_argument('output');parser.add_argument('--binary',required=True)
    args=parser.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/global_costmap/costmap']))
    selected={};zone=datetime.timezone(datetime.timedelta(hours=8))
    checkpoints=('12:52:47','13:01:50','13:03:32')
    while reader.has_next():
        _,data,stamp=reader.read_next()
        clock=datetime.datetime.fromtimestamp(stamp/1e9,zone).strftime('%H:%M:%S')
        for checkpoint in checkpoints:
            if clock<=checkpoint:selected[checkpoint]=(stamp,data)
        if clock>checkpoints[-1]:break
    results=[]
    for clock,(stamp,data) in selected.items():
        message=deserialize_message(data,OccupancyGrid);info=message.info
        q=info.origin.orientation
        assert abs(q.z)<1e-6 and abs(q.w-1.)<1e-6,'rotated grids require explicit conversion'
        w,h,res=info.width,info.height,info.resolution
        occupancy=np.asarray(message.data,dtype=np.int16).reshape(h,w)
        # OccupancyGrid is quantized; invert Nav2's translation table approximately.
        # Both algorithms receive EXACTLY the same reconstructed byte grid.
        costs=np.where(occupancy<0,255,np.where(occupancy==100,254,
            np.where(occupancy==99,253,np.where(occupancy>0,1+(occupancy-1)*251//97,0)))).astype(np.uint8)
        clearance=distance_transform_edt((occupancy>=0)&(occupancy<99))*res
        physical_clearance=distance_transform_edt((occupancy>=0)&(occupancy<100))*res
        geometry=StationGrid(message)
        scenarios=[('seat_to_left',(6.6,-.3),(7.2,5.7)),
                   ('seat_light_to_left',(1.4,-5.3),(7.2,5.7)),
                   ('ee_to_left',(2.,5.18),(7.2,5.7))]
        for name,start,goal in scenarios:
            cell=lambda p:(math.floor((p[0]-info.origin.position.x)/res),
                           math.floor((p[1]-info.origin.position.y)/res))
            a,b=cell(start),cell(goal)
            if not all(0<=x<w and 0<=y<h for x,y in (a,b)):continue
            # NavFnPlanner clears the starting robot cell, not all obstacles.
            grid=costs.copy();grid[a[1],a[0]]=0
            filename=out/(clock.replace(':','')+'_'+name+'.bin')
            filename.write_bytes(struct.pack('6i',w,h,*a,*b)+grid.tobytes())
            p=subprocess.run([args.binary,str(filename)],capture_output=True,text=True,check=True)
            comparisons=json.loads(p.stdout)
            for result in comparisons:
                cells=result['cells'];indices=np.asarray(cells,dtype=float)
                if len(indices):
                    distance=np.sqrt(np.sum(np.diff(indices,axis=0)**2,axis=1))*res
                    ii=np.clip(np.floor(indices[:,0]).astype(int),0,w-1)
                    jj=np.clip(np.floor(indices[:,1]).astype(int),0,h-1)
                    result['length_m']=float(distance.sum())
                    result['min_center_clearance_to_inscribed_or_unknown_m']=float(clearance[jj,ii].min())
                    result['unknown_path_samples']=int(np.sum(occupancy[jj,ii]<0))
                    result['min_center_clearance_to_occupied_or_unknown_m']=float(physical_clearance[jj,ii].min())
                result['world_path']=[[info.origin.position.x+x*res,info.origin.position.y+y*res]
                                      for x,y in cells]
                # A conservative forward-facing check, not an SE2 planner:
                # full 44x32 cm body +3 cm margin, shortest turn before translation.
                points=result['world_path'][::3]
                if result['world_path'] and (not points or points[-1]!=result['world_path'][-1]):
                    points.append(result['world_path'][-1])
                previous=(*start,math.pi if name=='seat_to_left' else 0.)
                blocked=0;blocked_details=[]
                for point in points:
                    if math.hypot(point[0]-previous[0],point[1]-previous[1])<.001:continue
                    heading=math.atan2(point[1]-previous[1],point[0]-previous[0])
                    turn=geometry.turn_clear(previous,heading)
                    translation=geometry.segment((*previous[:2],heading),(*point,heading))
                    if not turn or not translation:
                        blocked+=1
                        blocked_details.append(dict(at=list(previous[:2]),turn_clear=turn,
                            translation_clear=translation,target_distance_m=math.hypot(previous[0]-goal[0],previous[1]-goal[1])))
                    previous=(*point,heading)
                if points and not geometry.turn_clear(previous,0.):
                    blocked+=1;blocked_details.append(dict(at=list(previous[:2]),final_turn_blocked=True))
                result['conservative_forward_sweep_blocked_steps']=blocked if points else None
                result['blocked_forward_sweep_details']=blocked_details[:10]
                result.pop('cells')
            results.append(dict(snapshot=clock,scenario=name,start=start,goal=goal,results=comparisons))
    report=dict(offline=True,native_installed_navfn=True,quantized_costmap_reconstruction=True,
        footprint_sweep_validated=False,production_planner_changed=False,cases=results)
    (out/'comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    for r in results:
        print(json.dumps(dict(snapshot=r['snapshot'],scenario=r['scenario'],
            results=[{k:v for k,v in p.items() if k!='world_path'} for p in r['results']])))


if __name__=='__main__':main()
