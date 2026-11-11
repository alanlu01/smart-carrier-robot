"""Short-lived measured ray union for the actual sidestep rectangle."""
import math
import numpy as np


def intersections(sx,sy,vx,vy,bounds):
    enter=np.zeros_like(vx);leave=np.full_like(vx,np.inf)
    for origin,vector,low,high in ((sx,vx,bounds[0],bounds[1]),(sy,vy,bounds[2],bounds[3])):
        parallel=np.abs(vector)<1e-9
        with np.errstate(divide='ignore',invalid='ignore'):
            a,b=(low-origin)/vector,(high-origin)/vector
        near,far=np.minimum(a,b),np.maximum(a,b)
        near=np.where(parallel,0.,near)
        far=np.where(parallel,np.inf if low<=origin<=high else -1.,far)
        enter=np.maximum(enter,near);leave=np.minimum(leave,far)
    return leave>=np.maximum(enter,0.),leave


def aligned_swept_report(frames,base_pose,now,ros_now,direction,distance,half_x,half_y):
    """Keep the >=95% / <=4 degree budgets, with three independent frames.

    Rays are transformed with local odometry. Old rays must reach beyond the
    rectangle plus 2 m/s age allowance. Current/recent occlusion always wins.
    Persistent unknown beams never become measured free space.
    """
    report=dict(clear=False,reason='insufficient_recent_scan_evidence',evidence_frames=0,
                evidence_mode='odom_aligned_swept_rectangle')
    if (not frames or direction not in (-1,1) or min(distance,half_x,half_y)<=0
            or not all(math.isfinite(v) for v in (*base_pose,now,ros_now,distance,half_x,half_y))):return report
    latest=frames[-1]
    if not (-.1<=ros_now-latest.source_sec<=.30 and 0<=now-latest.received_at<=.30):
        return dict(report,reason='scan_stale_or_absent')
    x0,y0,yaw0=base_pose;c,s=math.cos(yaw0),math.sin(yaw0)
    def local(point):
        dx,dy=point[0]-x0,point[1]-y0
        return c*dx+s*dy,-s*dx+c*dy
    lx,ly=local(latest.sensor_pose)
    bounds=(-half_x-.08,half_x+.08,
            half_y if direction>0 else -half_y-distance-.08,
            half_y+distance+.08 if direction>0 else -half_y)
    angles=(np.arange(720)+.5)*math.pi/360-math.pi
    required,_=intersections(lx,ly,np.cos(angles),np.sin(angles),bounds)
    known=np.zeros(720,dtype=bool);used=0;sources=set();used_sources=[]
    mount0=local_mount(latest)
    for frame in reversed(frames):
        if frame.source_sec in sources:continue
        sources.add(frame.source_sec)
        age=max(0.,ros_now-frame.source_sec)
        if not (-.1<=ros_now-frame.source_sec<=.50 and 0<=now-frame.received_at<=.45):continue
        bx,by,byaw=frame.base_pose
        # Measured odom translation at the authorized .25 m/s cap, plus 3 cm
        # uncertainty. Large jumps/turns never authorize old free rays.
        displacement_limit=min(.15,.25*(age+.04)+.03)
        if (math.hypot(bx-x0,by-y0)>displacement_limit or abs(wrap(byaw-yaw0))>.10):continue
        mount=local_mount(frame)
        if math.hypot(mount[0]-mount0[0],mount[1]-mount0[1])>.005 or abs(wrap(mount[2]-mount0[2]))>.01:continue
        sx,sy=local(frame.sensor_pose)
        a=frame.angles+frame.sensor_pose[2]-yaw0;vx,vy=np.cos(a),np.sin(a)
        hit,leave=intersections(sx,sy,vx,vy,bounds)
        valid=np.isfinite(frame.ranges)&(frame.ranges>=frame.range_min)&(frame.ranges<=frame.range_max)
        if frame is latest:
            current=float(np.sum(valid&hit)/max(1,np.sum(hit)))
            report['current_valid_fraction']=round(current,4)
        if np.any(hit&valid&(frame.ranges<leave+.02)):
            return dict(report,reason='obstacle_or_occlusion' if frame is latest else 'recent_obstacle_or_occlusion')
        free=hit&valid&(frame.ranges>=leave+.02+2.*(age+.04))
        bins=np.floor((np.arctan2(sy+leave[free]*vy[free]-ly,sx+leave[free]*vx[free]-lx)+math.pi)*360/math.pi).astype(int)%720
        known[bins]=True;used+=1;used_sources.append(frame.source_sec)
    missing=required&~known
    run=peak=0
    for value in np.concatenate((missing,missing)):
        run=run+1 if value else 0;peak=max(peak,run)
    gap=min(360.,peak*.5+.5) if peak else 0.
    fraction=float(np.sum(known&required)/max(1,np.sum(required)))
    clear=(used>=3 and report.get('current_valid_fraction',0.)>=.80
           and fraction>=.95 and gap<=4.)
    return dict(report,clear=clear,reason='clear' if clear else 'unknown_coverage',
                evidence_frames=used,valid_fraction=round(fraction,4),missing_arc_deg=round(gap,3),
                evidence_deadline_sec=min(used_sources)+.50 if used_sources else ros_now)


def wrap(value):return math.atan2(math.sin(value),math.cos(value))


def local_mount(frame):
    bx,by,yaw=frame.base_pose;dx,dy=frame.sensor_pose[0]-bx,frame.sensor_pose[1]-by
    c,s=math.cos(yaw),math.sin(yaw)
    return c*dx+s*dy,-s*dx+c*dy,wrap(frame.sensor_pose[2]-yaw)
