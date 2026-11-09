"""Geometry, signal validation and frontier selection without ROS dependencies."""
import math
import heapq
from collections import deque


def wrap_delta(new, old):
    return (new - old + 32768) % 65536 - 32768


def locked_steering_profile(config):
    result=dict(config)
    result.update(servo_travel_us=500,front_center_us=1500,rear_center_us=1500)
    return result


def telemetry(line):
    from .motor_base import MOTOR
    if not line.startswith('H=') or not MOTOR.search(line):
        raise ValueError('Incomplete telemetry')
    f = dict(x.split('=', 1) for x in line.split() if '=' in x)
    if f.get('DRIVE') != '1' or int(f.get('FW', '0')) < 2:
        raise ValueError('Vehicle_Drive FW>=2 required')
    for key in ('E1', 'E2'):
        if not 0 <= int(f[key]) <= 65535:
            raise ValueError('Invalid encoder')
    if not all(math.isfinite(float(f[k])) for k in ('H','R','P','D1','D2')):
        raise ValueError('Nonfinite telemetry')
    return f


def steering(v, w, wheelbase, limit):
    if not all(math.isfinite(a) for a in (v,w,wheelbase,limit)) or wheelbase<=0 or limit<=0:
        raise ValueError('Invalid steering request or geometry')
    if abs(v) < .001:
        return 0.0
    angle = math.atan(wheelbase*w/(2*v))
    return max(-limit,min(limit,angle))


def require_calibration(c, require_verified=True):
    for key in ('wheelbase_m',
                'max_steer_rad','servo_travel_us','measured_speed_mps',
                'measured_duty_pct','stopping_distance_m'):
        v=c.get(key)
        if type(v) not in (int,float) or not math.isfinite(v) or v<=0:
            raise ValueError('Measure and set '+key)
    if require_verified and not c.get('verified',False):
        raise ValueError('Calibration must be physically verified before navigation')
    cap=c.get('max_duty_pct',100.)
    if type(cap) not in (int,float) or not math.isfinite(cap) or not 1<=cap<=100:
        raise ValueError('max_duty_pct must be 1..100')
    load=c.get('load_reference_current_a')
    if load is not None and (type(load) not in (int,float) or not math.isfinite(load) or load<=0):
        raise ValueError('load_reference_current_a must be measured, positive and finite')
    secondary=c.get('secondary_encoder')
    if secondary is not None:
        if (not isinstance(secondary,dict) or secondary.get('index') not in (1,2)
            or secondary['index']==c['encoder_index'] or secondary.get('sign') not in (-1,1)):
            raise ValueError('secondary_encoder must select the other encoder with measured sign')
        encoder_scale(secondary)
    encoder_scale(c)
    if c['encoder_index'] not in (1,2) or c['encoder_sign'] not in (-1,1):
        raise ValueError('Invalid encoder selection/sign')
    if not .01<=c['measured_speed_mps']<=.5:
        raise ValueError('Measured speed must be 0.01..0.5 m/s')
    if not 1<=c['measured_duty_pct']<=100 or not 0<c['max_steer_rad']<.65:
        raise ValueError('Duty/steering outside supported bounds')
    if c['servo_travel_us']>500:
        raise ValueError('Servo travel exceeds 500 us')
    for axle in ('front','rear'):
        if c[axle+'_sign'] not in (-1,1): raise ValueError('Invalid servo sign')
        center=c[axle+'_center_us']
        if not 1000+c['servo_travel_us']<=center<=2000-c['servo_travel_us']:
            raise ValueError('Servo pulse limits exceeded')
    for key in ('motor1_sign','motor2_sign','imu_yaw_sign'):
        if c[key] not in (-1,1): raise ValueError('Invalid '+key)
    for key in ('imu_yaw_offset_rad','laser_x_m','laser_y_m','laser_z_m','laser_yaw_rad','clearance_margin_m'):
        if type(c.get(key)) not in (int,float) or not math.isfinite(c[key]):
            raise ValueError('Invalid '+key)
    if not .1<=c['clearance_margin_m']<=1:
        raise ValueError('Clearance margin must be 0.1..1 m')
    if not 0<c['wheelbase_m']<=1 or not 25<=c['servo_travel_us']<=500:
        raise ValueError('Unsupported wheelbase or servo travel')
    if not isinstance(c['laser_frame'],str) or not c['laser_frame']:
        raise ValueError('Set the actual /scan header.frame_id')
    return c


def encoder_scale(c):
    """Counts/metre may be measured directly without wheel/gear specifications."""
    scale=c.get('counts_per_meter')
    if scale is None:
        diameter=c.get('wheel_diameter_m');counts=c.get('counts_per_wheel_rev')
        if not all(type(v) in (int,float) and math.isfinite(v) and v>0 for v in (diameter,counts)):
            raise ValueError('Measure counts_per_meter, or wheel diameter and counts_per_wheel_rev')
        scale=counts/(math.pi*diameter)
    if type(scale) not in (int,float) or not math.isfinite(scale) or scale<=0:
        raise ValueError('Invalid counts_per_meter')
    return scale


def frontier_goals(data,width,height,resolution,origin,robot,clearance=.38):
    """Reachable free-space boundary clusters; never goal into unknown space.

    A disk is used to inflate occupied cells for candidate screening. Nav2
    still checks orientation, footprint, curvature and reachability.
    """
    if len(data)!=width*height or resolution<=0: return []
    ox,oy=origin
    rx,ry=robot
    sx,sy=int((rx-ox)/resolution),int((ry-oy)/resolution)
    if not(0<=sx<width and 0<=sy<height): return []
    def adjacent(k):
        x,y=k%width,k//width
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            a,b=x+dx,y+dy
            if 0<=a<width and 0<=b<height: yield b*width+a
    # Multi-source distance through the grid; occupied inflation excludes
    # diagonal corner cutting conservatively via an 8-connected distance.
    distance=[float('inf')]*len(data);heap=[]
    for k,v in enumerate(data):
        if v>=50: distance[k]=0;heap.append((0,k))
    heapq.heapify(heap)
    while heap:
        dist,k=heapq.heappop(heap)
        if dist!=distance[k] or dist>=clearance: continue
        x,y=k%width,k//width
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            a,b=x+dx,y+dy
            if not(0<=a<width and 0<=b<height):continue
            # Chebyshev distance is <= Euclidean distance, so this inflates
            # conservatively instead of underestimating diagonal clearance.
            j=b*width+a;nd=dist+resolution
            if nd<distance[j]:distance[j]=nd;heapq.heappush(heap,(nd,j))
    start=sy*width+sx
    if data[start]<0 or data[start]>=50 or distance[start]<clearance:return []
    reachable={start};queue=deque([start]);travel={start:0.}
    while queue:
        k=queue.popleft()
        for j in adjacent(k):
            if j not in reachable and 0<=data[j]<50 and distance[j]>=clearance:
                reachable.add(j);travel[j]=travel[k]+resolution;queue.append(j)
    boundaries={k for k in reachable if any(data[j]<0 for j in adjacent(k))}
    # Put the actual goal back from the unknown boundary so the entire robot
    # can stay in observed space. A frontier cell itself is not a safe pose.
    unknown_distance=[float('inf')]*len(data);q=deque()
    for k in reachable:
        if any(data[j]<0 for j in adjacent(k)):
            unknown_distance[k]=resolution/2;q.append(k)
    while q:
        k=q.popleft()
        if unknown_distance[k]>=clearance+resolution:continue
        x,y=k%width,k//width
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            a,b=x+dx,y+dy
            if not(0<=a<width and 0<=b<height):continue
            j=b*width+a
            if j in reachable and unknown_distance[j]>unknown_distance[k]+resolution:
                unknown_distance[j]=unknown_distance[k]+resolution;q.append(j)
    safe_goals=[k for k in reachable if unknown_distance[k]>=clearance]
    result=[]
    while boundaries:
        seed=boundaries.pop();cluster=[seed];queue=deque([seed])
        while queue:
            for j in adjacent(queue.popleft()):
                if j in boundaries:boundaries.remove(j);cluster.append(j);queue.append(j)
        if len(cluster)*resolution<.20:continue
        cx=sum(k%width for k in cluster)/len(cluster)
        cy=sum(k//width for k in cluster)/len(cluster)
        # The centroid of a ring-shaped frontier can be the robot's own
        # position. Anchor to an actual boundary cell before backing off.
        representative=min(cluster,key=lambda a:(a%width-cx)**2+(a//width-cy)**2)
        cx,cy=representative%width,representative//width
        if not safe_goals:continue
        k=min(safe_goals,key=lambda a:(a%width-cx)**2+(a//width-cy)**2)
        if math.hypot(k%width-cx,k//width-cy)*resolution>max(1.,2*clearance):continue
        x=ox+(k%width+.5)*resolution;y=oy+(k//width+.5)*resolution
        if math.hypot(x-rx,y-ry)<.5:continue
        yaw=math.atan2(y-ry,x-rx)
        score=len(cluster)*resolution/(1+travel[k])
        result.append(dict(x=x,y=y,yaw=yaw,score=score,size=len(cluster)))
    return sorted(result,key=lambda g:g['score'],reverse=True)


def choose_goal(candidates,memory,now,robot_yaw=None,exclusion_radius=.35):
    if robot_yaw is not None:
        candidates=sorted(candidates,key=lambda g:g['score']/(1+2*abs(math.atan2(
            math.sin(g['yaw']-robot_yaw),math.cos(g['yaw']-robot_yaw)))),reverse=True)
    for goal in candidates:
        if not any(m['until']>now and math.hypot(goal['x']-m['x'],goal['y']-m['y'])<exclusion_radius for m in memory):
            return goal
    return None
