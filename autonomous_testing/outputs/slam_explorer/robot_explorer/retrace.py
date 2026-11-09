"""Recorded-route recovery and a short straight backup checked by the bridge."""
import math
from collections import deque

MAX_RETRACE_DISTANCE=2.0
MAX_RETRACE_SECONDS=15.0
BRIDGE_REVERSE_DISTANCE=2.15
BRIDGE_REVERSE_SECONDS=16.0


class RouteDeviation(ValueError):
    pass


def straight_reverse_command(anchor,current,speed):
    if not all(math.isfinite(v) for v in (*anchor[:3],*current[:3],speed)) or speed<=0:
        raise ValueError('Invalid straight reverse pose or speed')
    # Track displacement from one fixed anchor; never reset it each cycle.
    if math.dist(anchor[:2],current[:2])>=1.:return 0.,0.,True
    return -speed,0.,False


def angle(a):return math.atan2(math.sin(a),math.cos(a))


def short_backup(current):
    """A 1 m straight target; bridge rear coverage/clearance gates every command."""
    x,y,yaw,stamp=current
    if not all(math.isfinite(v) for v in current):return []
    return [(x-.025*i*math.cos(yaw),y-.025*i*math.sin(yaw),yaw,stamp)
            for i in range(41)]


class Trail:
    def __init__(self):self.points=deque(maxlen=1000)
    def add(self,x,y,yaw,stamp):
        if not all(math.isfinite(v) for v in (x,y,yaw,stamp)):return
        if self.points:
            previous=self.points[-1]
            distance=math.hypot(x-previous[0],y-previous[1])
            if distance>.35 or stamp<previous[3]:self.points.clear()
            elif distance<.035:return
        self.points.append((x,y,yaw,stamp))
    def reverse(self,current,now,limit=MAX_RETRACE_DISTANCE):
        if not self.points or now-self.points[-1][3]>45:return []
        if math.hypot(current[0]-self.points[-1][0],current[1]-self.points[-1][1])>.12:return []
        path=[current];length=0.
        for p in reversed(self.points):
            if now-p[3]>45:break
            last=path[-1];dx,dy=p[0]-last[0],p[1]-last[1];step=math.hypot(dx,dy)
            if step<.02:continue
            # Recorded direction must agree with backing, not a sideways jump.
            if dx*math.cos(last[2])+dy*math.sin(last[2])>=0:break
            if abs(angle(p[2]-last[2]))>.35:break
            if length+step>limit:
                f=(limit-length)/step
                path.append((last[0]+dx*f,last[1]+dy*f,last[2]+angle(p[2]-last[2])*f,p[3]));length=limit;break
            path.append(p);length+=step
        return path if length>=.18 else []


def reverse_command(path,current,speed,wheelbase,max_angle):
    """Pure-pursuit of recorded points, with a negative longitudinal command."""
    if not path:return 0.,0.,True
    # Distance to the continuous route, not only its sampled breadcrumbs.
    # Adjacent valid breadcrumbs may be up to 0.35 m apart.
    distances=[]
    for i,(a,b) in enumerate(zip(path,path[1:])):
        dx,dy=b[0]-a[0],b[1]-a[1];length2=dx*dx+dy*dy
        t=0. if length2==0 else max(0.,min(1.,((current[0]-a[0])*dx+(current[1]-a[1])*dy)/length2))
        distances.append((math.hypot(current[0]-a[0]-t*dx,current[1]-a[1]-t*dy),i))
    distance,closest=min(distances) if distances else (math.dist(path[0][:2],current[:2]),0)
    if distance>.15:
        raise RouteDeviation(f'Reverse path deviation {distance:.3f} m exceeds 0.150 m')
    if math.hypot(path[-1][0]-current[0],path[-1][1]-current[1])<.055:return 0.,0.,True
    target=None
    for p in path[closest:]:
        dx,dy=p[0]-current[0],p[1]-current[1]
        if math.cos(current[2])*dx+math.sin(current[2])*dy>=-.005:continue
        target=p
        if math.hypot(p[0]-current[0],p[1]-current[1])>=.16:break
    # No remaining point behind means stop this recovery and replan. In
    # particular, odometry can pass the endpoint between updates; do not turn
    # that normal completion into a fault or command reverse toward a point ahead.
    if target is None:return 0.,0.,True
    dx,dy=target[0]-current[0],target[1]-current[1]
    x=math.cos(current[2])*dx+math.sin(current[2])*dy
    y=-math.sin(current[2])*dx+math.cos(current[2])*dy
    curvature=2*y/max(.001,x*x+y*y)
    cap=2*math.tan(max_angle)/wheelbase
    curvature=max(-cap,min(cap,curvature))
    return -speed,-speed*curvature,False


def rear_coverage_details(scan,cfg,v=-.15,w=0.):
    # Straight reverse needs coverage of the corridor beyond the rear bumper,
    # not lateral sectors outside that corridor. Turning retains the full rear
    # hemisphere requirement; obstacle_stop still checks all measured returns.
    radius=math.hypot(.3048,.3048)/2+cfg.get('clearance_margin_m',.12)
    rear_plane=.3048/2+cfg.get('laser_x_m',0.)
    half_angle=(90. if abs(w)>1e-6 or rear_plane<=0 else
                math.degrees(math.atan2(radius+abs(cfg.get('laser_y_m',0.)),rear_plane)))
    # Keep the same angular corridor; divide it into exactly 20 sectors.
    required=set(range(20))
    width=2*half_angle/20
    bins=set()
    invalid=0
    for i,r in enumerate(scan.ranges):
        a=scan.angle_min+i*scan.angle_increment+cfg['laser_yaw_rad']
        relative=math.degrees(angle(a-math.pi))
        if not -half_angle<=relative<=half_angle:continue
        b=min(19,int((relative+half_angle)/width))
        if math.isfinite(r) and scan.range_min<r<scan.range_max:
            bins.add(b)
        elif b in required:invalid+=1
    missing=sorted(required-bins)
    known=len(required & bins)
    # User-selected tolerance: one of twenty sectors is sufficient. Missing
    # rays remain unknown; this threshold is not proof that the corridor is free.
    sufficient=bool(required) and known>=1
    return dict(complete=not missing,sufficient=sufficient,minimum_fraction=.05,
                known_bins=known,required_bins=len(required),
                missing_degrees=[round((180-half_angle+b*width)%360,1) for b in missing],
                sector_width_deg=round(width,2),invalid_samples=invalid,
                half_angle_deg=round(half_angle,1))


def rear_coverage(scan,cfg,v=-.15,w=0.):
    return rear_coverage_details(scan,cfg,v,w)['sufficient']
