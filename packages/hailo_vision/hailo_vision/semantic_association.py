"""Small bounded target histories and diagnostic-only range cluster summaries."""
import math
from collections import deque


class TargetRangeHistory:
    """Associate only one unambiguous nearby angle/width track per frame."""
    def __init__(self):
        self.tracks, self.used, self.serial = {}, set(), 0

    def begin_frame(self, now):
        self.used.clear()
        self.tracks = {k:v for k,v in self.tracks.items() if 0 <= now-v['at'] <= .5}

    def observe(self, target, distance, now):
        angle, width = float(target['angle']), float(target['bbox_width'])
        candidates = [k for k,v in self.tracks.items() if k not in self.used
            and v['class'] == target['id'] and abs(angle-v['angle']) <= 6.
            and .6 <= width/max(1.,v['width']) <= 1.67]
        if len(candidates) == 1:
            key = candidates[0]
        else:
            self.serial += 1
            key = self.serial
            if len(self.tracks) >= 64:
                self.tracks.pop(min(self.tracks,key=lambda k:self.tracks[k]['at']))
            self.tracks[key] = {'class':target['id'],'ranges':deque(maxlen=5)}
        track = self.tracks[key]
        track.update(angle=angle,width=width,at=now)
        track['ranges'].append(distance)
        self.used.add(key)
        return min(track['ranges']), key


def range_association(target, scan):
    """Keep the existing second-nearest rule; expose support before retuning it."""
    angle = math.radians((180.-float(target['angle'])) % 360.)
    center = int(((angle-scan.angle_min+2*math.pi) % (2*math.pi))/scan.angle_increment)
    half = max(float(target['bbox_width'])/640.*68./2.*1.2,3.)
    window = int(math.radians(half)/abs(scan.angle_increment))
    values = []
    for offset in range(-window,window+1):
        index = (center+offset) % len(scan.ranges)
        value = scan.ranges[index]
        if math.isfinite(value) and .35 < value < scan.range_max:
            values.append((value,index))
    if not values:
        return None, {'reason':'no_valid_range'}
    values.sort()
    distance,index = values[min(1,len(values)-1)]
    # Range clusters are diagnostics only until an image-backed field test.
    first = values[0][0]
    support = sum(value <= first+.10 for value,_ in values)
    return distance, dict(reason='second_nearest',distance_m=round(distance,3),
        valid_beams=len(values),near_cluster_support=support,
        selected_laser_angle_deg=round(math.degrees(scan.angle_min+index*scan.angle_increment),2),
        search_half_angle_deg=round(half,2))
