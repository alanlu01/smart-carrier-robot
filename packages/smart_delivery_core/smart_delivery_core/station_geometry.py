"""Bounded station candidates, using occupied cells and the complete body sweep."""
import math


def angle_error(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


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
                if self.clear(point, rotation=True) and self.segment(current,point):
                    candidates.append(point)
        return min(candidates, key=lambda p:math.hypot(p[0]-target[0],p[1]-target[1])) if candidates else None
