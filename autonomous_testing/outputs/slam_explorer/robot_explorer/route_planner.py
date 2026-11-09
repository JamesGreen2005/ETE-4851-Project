"""Three-stage beam search over steering-feasible, collision-checked arcs."""
import math
from .footprint import footprint_known_map,cell_intersects_body


def viewpoint_goals(data,width,height,resolution,origin,pose,wheelbase,max_steer,margin=.12,visits=()):
    if resolution<=0 or len(data)!=width*height or wheelbase<=0:
        return [],dict(error='Invalid map or geometry',valid_viewpoints=0)
    data,known_cells=footprint_known_map(data,width,height,resolution,origin,pose)
    ox,oy=origin;rx,ry,yaw=pose;body=math.hypot(.1524,.1524)
    step=min(.025,resolution*.4);cache={}
    def value(x,y):
        gx,gy=math.floor((x-ox)/resolution),math.floor((y-oy)/resolution)
        return data[gy*width+gx] if 0<=gx<width and 0<=gy<height else -1
    def clear(x,y,padding,heading=yaw):
        radius=math.sqrt(2)*(.1524+padding+step/2)
        key=(round(x,5),round(y,5),round(heading,5),round(padding,5))
        if key in cache:return cache[key]
        for gy in range(math.floor((y-radius-oy)/resolution),math.floor((y+radius-oy)/resolution)+1):
            for gx in range(math.floor((x-radius-ox)/resolution),math.floor((x+radius-ox)/resolution)+1):
                cx,cy=ox+(gx+.5)*resolution,oy+(gy+.5)*resolution
                v=data[gy*width+gx] if 0<=gx<width and 0<=gy<height else -1
                # Unknown space is forbidden for the body, but is not an
                # occupied obstacle in the optional extra clearance margin.
                sweep=0. if x==rx and y==ry and heading==yaw else step/2
                extent=.1524+(padding if v>=50 else 0.)+sweep
                if (v<0 or v>=50) and cell_intersects_body(cx,cy,resolution,(x,y,heading),extent):
                    cache[key]=False;return False
        cache[key]=True;return True
    full=clear(rx,ry,margin)
    if not full and not clear(rx,ry,0.):
        return [],dict(error='Current body footprint intersects unknown or occupied map cells',valid_viewpoints=0)
    diagnostic=dict(candidates=0,rejected_unknown_or_obstacle=0,planner='steering_lattice',depth=3,known_under_body_cells=known_cells)
    info_cache={}
    def information(x,y):
        key=(round(x/resolution),round(y/resolution))
        if key not in info_cache:
            gain=0
            for i in range(24):
                a=i*math.pi/12
                for j in range(1,25):
                    v=value(x+j*.05*math.cos(a),y+j*.05*math.sin(a))
                    if v>=50:break
                    if v<0:gain+=1;break
            info_cache[key]=gain
        return info_cache[key]
    def score(n):
        x,y,heading,length,direction,turn,segments=n
        revisits=sum(math.exp(-((x-v[0])**2+(y-v[1])**2)/.16) for v in visits)
        switches=sum(a[0]!=b[0] for a,b in zip(segments,segments[1:]))
        return (1+information(x,y)*.18+min(math.hypot(x-rx,y-ry),1.4)*1.3
                -.2*sum(abs(t)*d for _,t,d in segments)-.5*min(revisits,8.)
                -(.2 if direction<0 else 0)-.12*length-.3*switches)
    def extend(n,direction,turn,distance):
        x,y,heading,length,_,_,segments=n;c=2*math.tan(turn*max_steer)/wheelbase
        diagnostic['candidates']+=1
        for j in range(1,math.ceil(distance/step)+1):
            travelled=min(distance,j*step);s=direction*travelled
            bx=s if abs(c)<1e-9 else math.sin(c*s)/c
            by=0 if abs(c)<1e-9 else (1-math.cos(c*s))/c
            px=x+math.cos(heading)*bx-math.sin(heading)*by
            py=y+math.sin(heading)*bx+math.cos(heading)*by
            padding=margin if full else margin*min(1.,(length+travelled)/.4)
            if not clear(px,py,padding,heading+c*direction*travelled):
                diagnostic['rejected_unknown_or_obstacle']+=1;return None
        return (px,py,heading+c*direction*distance,length+distance,direction,turn,segments+[(direction,turn,distance)])
    root=(rx,ry,yaw,0.,1,0.,[]);result=[];beam=[]
    for direction in (1,-1):
        for turn in (0.,-.5,.5,-1.,1.):
            for distance in (.25,.5,.9,1.4):
                n=extend(root,direction,turn,distance)
                if n:result.append(n);beam.append(n)
    for depth in range(2):
        chosen=[];bins=set()
        for n in sorted(beam,key=score,reverse=True):
            key=(round(n[0]/.2),round(n[1]/.2),round(n[2]/.35),n[4])
            if key not in bins:chosen.append(n);bins.add(key)
            if len(chosen)>=16:break
        beam=[]
        for n in chosen:
            changes=sum(a[0]!=b[0] for a,b in zip(n[6],n[6][1:]))
            for direction in (n[4],-n[4]):
                if direction!=n[4] and changes:continue
                for turn in (0.,-1.,1.):
                    child=extend(n,direction,turn,.45)
                    if child:result.append(child);beam.append(child)
    goals=[]
    for n in sorted(result,key=score,reverse=True):
        x,y,heading,distance,direction,turn,segments=n
        if math.hypot(x-rx,y-ry)<.2:continue
        if any(math.hypot(x-g['x'],y-g['y'])<.15 and abs(math.atan2(math.sin(heading-g['yaw']),math.cos(heading-g['yaw'])))<.4 for g in goals):continue
        goals.append(dict(x=x,y=y,yaw=heading,score=score(n),size=information(x,y),source='steering_lattice',
                          distance=distance,turn=turn,direction=direction,segments=segments))
    diagnostic['valid_viewpoints']=len(goals)
    return goals,diagnostic
