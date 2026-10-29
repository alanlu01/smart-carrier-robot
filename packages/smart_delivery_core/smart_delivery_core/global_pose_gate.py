"""Bounded geometric cross-check of a global AMCL candidate, not a posterior."""
import math
from smart_delivery_core.localization_health import select_scan_samples


def ray_consistency(grid, scan, laser_pose):
    """Check <=60 endpoints and <=512 cells/ray using only the static map."""
    report = dict(verified=False, reason='insufficient_geometry', rays=0)
    resolution = float(grid.info.resolution)
    if resolution <= 0 or not all(math.isfinite(v) for v in (*laser_pose,resolution)):
        return report
    width,height = int(grid.info.width),int(grid.info.height)
    if width <= 0 or height <= 0 or len(grid.data) != width*height:
        return report
    origin = grid.info.origin
    q = origin.orientation
    if (not all(math.isfinite(v) for v in (origin.position.x,origin.position.y,
            q.x,q.y,q.z,q.w,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max))
            or scan.angle_increment==0 or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):
        return dict(report,reason='invalid_geometry')
    angle = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    co,si = math.cos(angle),math.sin(angle)
    dx,dy = laser_pose[0]-origin.position.x,laser_pose[1]-origin.position.y
    sx,sy = co*dx+si*dy,-si*dx+co*dy
    selection = select_scan_samples(scan.ranges,scan.range_min,scan.range_max,.55,60,12)
    hits=conflicts=0
    sectors={}
    for index,distance in selection.samples:
        theta=laser_pose[2]-angle+scan.angle_min+index*scan.angle_increment
        xdir,ydir=math.cos(theta),math.sin(theta)
        ex,ey=sx+distance*xdir,sy+distance*ydir
        gx,gy=math.floor(ex/resolution),math.floor(ey/resolution)
        matched=any(grid.data[yy*width+xx]>=65
                    for xx in range(max(0,gx-2),min(width,gx+3))
                    for yy in range(max(0,gy-2),min(height,gy+3)))
        hits+=matched
        sector=index*12//len(scan.ranges)
        sectors[sector]=sectors.get(sector,0)+matched
        # A return beyond a fixed wall contradicts the candidate. Unknown map
        # cells are never positive support; endpoint support is required too.
        # Grid DDA visits every crossed cell; float world offsets can skip a
        # thin wall at cell boundaries. Diagonal rays remain bounded as well.
        ix,iy=math.floor(sx/resolution),math.floor(sy/resolution)
        stepx,stepy=(1 if xdir>=0 else -1),(1 if ydir>=0 else -1)
        tx=((ix+(stepx>0))*resolution-sx)/xdir if abs(xdir)>1e-12 else math.inf
        ty=((iy+(stepy>0))*resolution-sy)/ydir if abs(ydir)>1e-12 else math.inf
        dtx=resolution/abs(xdir) if abs(xdir)>1e-12 else math.inf
        dty=resolution/abs(ydir) if abs(ydir)>1e-12 else math.inf
        entered=0.
        for _ in range(512):
            if entered>=min(distance-.30,8.):
                break
            if entered>=.30 and 0<=ix<width and 0<=iy<height and grid.data[iy*width+ix]>=65:
                conflicts+=1
                break
            if tx<ty:
                entered=tx;tx+=dtx;ix+=stepx
            else:
                entered=ty;ty+=dty;iy+=stepy
        else:
            return dict(report,reason='ray_budget_exceeded')
    n=len(selection.samples)
    if n < 20:
        return report
    fraction=conflicts/n
    endpoint=hits/n
    supported=sum(value>=2 for value in sectors.values())
    verified=fraction<=.30 and endpoint>=.55 and supported>=6
    report.update(verified=verified, reason='consistent' if verified else 'map_ray_contradiction_or_weak_support',
                  rays=n, conflict_fraction=round(fraction,5), endpoint_match=round(endpoint,5),
                  supported_sectors=supported, max_conflict_fraction=.30,
                  min_endpoint_match=.55, min_supported_sectors=6)
    return report
