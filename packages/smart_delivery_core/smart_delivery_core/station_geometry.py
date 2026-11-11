"""Bounded station candidates, using occupied cells and the complete body sweep."""
import math


def angle_error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def convex_hull(points):
    points=sorted(set(points))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lower,upper=[],[]
    for p in points:
        while len(lower)>1 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper)>1 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]


def rectangle(pose,hx=.25,hy=.19):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*dx-s*dy,y+s*dx+c*dy) for dx,dy in ((hx,hy),(hx,-hy),(-hx,-hy),(-hx,hy))]


class StationGrid:
    """OccupancyGrid snapshot; unknown, off-map and malformed cells are blocked."""

    def __init__(self, message):
        info = message.info
        self.resolution, self.width, self.height = info.resolution, info.width, info.height
        self.x, self.y = info.origin.position.x, info.origin.position.y
        q = info.origin.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.data = message.data
        if (not math.isfinite(self.resolution) or self.resolution <= 0
                or self.width*self.height != len(self.data) or not self.data
                or not all(math.isfinite(v) for v in (self.x, self.y, self.yaw))):
            raise ValueError('invalid station costmap')

    def clear(self, pose, rotation=False):
        # Add 3 cm to the real 44 x 32 cm body. Test cell squares, not just
        # centers, to avoid losing a thin chair leg along the sampled boundary.
        x, y, yaw = pose
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        gx, gy = c*(x-self.x)+s*(y-self.y), -s*(x-self.x)+c*(y-self.y)
        radius = math.hypot(.25, .19)
        half_cell = self.resolution/2
        c, s = math.cos(yaw-self.yaw), math.sin(yaw-self.yaw)
        ac, ass = abs(c), abs(s)
        extent_x = radius if rotation else .25*ac+.19*ass
        extent_y = radius if rotation else .25*ass+.19*ac
        lo_x, hi_x = math.floor((gx-extent_x)/self.resolution-1e-9), math.floor((gx+extent_x)/self.resolution+1e-9)
        lo_y, hi_y = math.floor((gy-extent_y)/self.resolution-1e-9), math.floor((gy+extent_y)/self.resolution+1e-9)
        if lo_x < 0 or lo_y < 0 or hi_x >= self.width or hi_y >= self.height:
            return False
        for row in range(lo_y, hi_y+1):
            for col in range(lo_x, hi_x+1):
                value = self.data[row*self.width+col]
                # In nav_msgs/OccupancyGrid the lethal cost is 100. Inscribed
                # inflation (99) is not a physical wall; full-body checks below
                # and MPPI retain footprint collision and inflation penalties.
                if 0 <= value < 100:
                    continue
                dx, dy = (col+.5)*self.resolution-gx, (row+.5)*self.resolution-gy
                if rotation:
                    # Distance to the square itself, not its circumscribed
                    # circle. Preserve the complete rotational body sweep.
                    inside = (max(abs(dx)-half_cell,0.)**2
                              + max(abs(dy)-half_cell,0.)**2 <= radius**2+1e-12)
                else:
                    # Separating axes for an oriented body and axis-aligned
                    # cell: both grid axes AND both body axes must overlap.
                    # Using the cell half-diagonal on each axis falsely blocked
                    # the recorded (-3,5) station despite >3 cm clearance.
                    inside = (abs(dx) <= extent_x+half_cell+1e-12
                              and abs(dy) <= extent_y+half_cell+1e-12
                              and abs(c*dx+s*dy) <= .25+half_cell*(ac+ass)+1e-12
                              and abs(-s*dx+c*dy) <= .19+half_cell*(ac+ass)+1e-12)
                if inside:
                    return False
        return True

    def segment(self, start, end):
        if abs(angle_error(start[2],end[2])) > .12:
            return False
        steps = max(1, math.ceil(math.hypot(end[0]-start[0], end[1]-start[1])/.02))
        return all(self.clear((start[0]+(end[0]-start[0])*i/steps,
                               start[1]+(end[1]-start[1])*i/steps, end[2]))
                   for i in range(steps+1))

    def polygon_clear(self, world_points):
        """Exact separating axes for a convex sweep and occupied cell squares."""
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        points=convex_hull([(c*(x-self.x)+s*(y-self.y),-s*(x-self.x)+c*(y-self.y))
                            for x,y in world_points])
        if len(points)<3:return False
        xs,ys=zip(*points);res=self.resolution;half=res/2
        lo_x,hi_x=math.floor(min(xs)/res-1e-9),math.floor(max(xs)/res+1e-9)
        lo_y,hi_y=math.floor(min(ys)/res-1e-9),math.floor(max(ys)/res+1e-9)
        if lo_x<0 or lo_y<0 or hi_x>=self.width or hi_y>=self.height:return False
        axes=[]
        for a,b in zip(points,points[1:]+points[:1]):
            nx,ny=-(b[1]-a[1]),b[0]-a[0]
            projections=[nx*x+ny*y for x,y in points]
            axes.append((nx,ny,min(projections),max(projections),half*(abs(nx)+abs(ny))))
        for row in range(lo_y,hi_y+1):
            for col in range(lo_x,hi_x+1):
                value=self.data[row*self.width+col]
                if 0<=value<100:continue
                x,y=(col+.5)*res,(row+.5)*res
                if all(not (nx*x+ny*y+pad<lo-1e-12 or nx*x+ny*y-pad>hi+1e-12)
                       for nx,ny,lo,hi,pad in axes):return False
        return True

    def turn_clear(self, pose, target_yaw):
        """Check the required shortest turn, not an unnecessary complete circle."""
        delta=angle_error(target_yaw,pose[2]);steps=max(1,math.ceil(abs(delta)/.10))
        step=delta/steps
        # Endpoint rectangle hull plus this sagitta covers the continuous arc
        # of every padded-body corner between samples, not only sampled poses.
        pad=math.hypot(.25,.19)*(1-math.cos(abs(step)/2))+1e-9
        for i in range(steps):
            corners=rectangle((pose[0],pose[1],pose[2]+step*i))+rectangle((pose[0],pose[1],pose[2]+step*(i+1)))
            if not self.polygon_clear([(x+dx,y+dy) for x,y in corners
                                       for dx,dy in ((pad,pad),(pad,-pad),(-pad,pad),(-pad,-pad))]):return False
        return True

    def aligned_route(self, start, end):
        """Conservative straight corridor for ordinary, forward-facing travel."""
        dx,dy=end[0]-start[0],end[1]-start[1]
        if math.hypot(dx,dy)<.10:return self.turn_clear(start,end[2]) and self.clear(end)
        bearing=math.atan2(dy,dx)
        deviation=max(abs(angle_error(start[2],bearing)),abs(angle_error(end[2],bearing)))
        if deviation>.15:return False
        # Bound all intermediate headings, preserving the full 3 cm margin.
        hx=.25*math.cos(deviation)+.19*math.sin(deviation)
        hy=.19*math.cos(deviation)+.25*math.sin(deviation)
        return (self.turn_clear(start,bearing) and self.turn_clear(end,bearing)
                and self.polygon_clear(rectangle((start[0],start[1],bearing),hx,hy)
                                       +rectangle((end[0],end[1],bearing),hx,hy)))

    def normal_route_safe(self, start, end):
        return ((self.clear(start,rotation=True) and self.clear(end,rotation=True))
                or self.aligned_route(start,end))

    def staging(self, target, current):
        """Try all four body directions; never invent permanent site coordinates."""
        candidates = []
        for distance in (.35, .50):
            for direction in (0., math.pi/2, -math.pi/2, math.pi):
                a = target[2]+direction
                point = (target[0]+distance*math.cos(a), target[1]+distance*math.sin(a), target[2])
                if self.clear(point, rotation=True) and self.segment(point, target):
                    candidates.append(point)
        return min(candidates, key=lambda p:math.hypot(p[0]-current[0],p[1]-current[1])) if candidates else None

    def departure(self, current, target):
        candidates = []
        for distance in (.35, .50):
            for direction in (0., math.pi/2, -math.pi/2, math.pi):
                a = current[2]+direction
                point = (current[0]+distance*math.cos(a),current[1]+distance*math.sin(a),current[2])
                if (self.clear(point,rotation=True) or self.aligned_route(point,target)) and self.segment(current,point):
                    candidates.append(point)
        return min(candidates, key=lambda p:math.hypot(p[0]-target[0],p[1]-target[1])) if candidates else None
