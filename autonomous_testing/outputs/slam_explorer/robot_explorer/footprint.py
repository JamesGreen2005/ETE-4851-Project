"""Grid-resolution footprint evidence, without erasing occupied observations."""
import math


def observed_rays(data,width,height,resolution,origin,sensor,endpoints):
    """Refresh unknown cells only along finite observed LiDAR rays.

    Occupied map cells dominate. Missing/infinite rays never clear anything.
    Endpoints are marked as obstacles; this overlay does not alter raw SLAM.
    """
    result=list(data);ox,oy=origin
    def cell(x,y):return math.floor((x-ox)/resolution),math.floor((y-oy)/resolution)
    sx,sy=cell(*sensor)
    if not(0<=sx<width and 0<=sy<height):return result
    for px,py in endpoints:
        if not math.isfinite(px+py):continue
        ex,ey=cell(px,py);x,y=sx,sy;dx=abs(ex-x);dy=-abs(ey-y)
        stepx=1 if x<ex else -1;stepy=1 if y<ey else -1;err=dx+dy
        for _ in range(width+height):
            if not(0<=x<width and 0<=y<height):break
            k=y*width+x
            if x==ex and y==ey:
                result[k]=100;break
            if result[k]>=50:break
            if result[k]<0:result[k]=0
            twice=2*err
            if twice>=dy:err+=dy;x+=stepx
            if twice<=dx:err+=dx;y+=stepy
    return result


def cell_intersects_body(cx,cy,resolution,pose,half=.1524):
    x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);dx,dy=cx-x,cy-y
    h=resolution/2
    # Separating-axis test: world grid axes and the two chassis axes.
    if abs(dx)>h+half*(abs(c)+abs(s)) or abs(dy)>h+half*(abs(c)+abs(s)):return False
    return abs(c*dx+s*dy)<=half+h*(abs(c)+abs(s)) and abs(-s*dx+c*dy)<=half+h*(abs(c)+abs(s))


def footprint_known_map(data,width,height,resolution,origin,pose):
    result=list(data);changed=0
    if resolution<=0 or len(result)!=width*height:return result,changed
    x,y,_=pose;ox,oy=origin;radius=math.sqrt(2)*.1524
    for gy in range(max(0,math.floor((y-radius-oy)/resolution)),min(height,math.floor((y+radius-oy)/resolution)+1)):
        for gx in range(max(0,math.floor((x-radius-ox)/resolution)),min(width,math.floor((x+radius-ox)/resolution)+1)):
            k=gy*width+gx
            if result[k]<0 and cell_intersects_body(ox+(gx+.5)*resolution,oy+(gy+.5)*resolution,resolution,pose):
                result[k]=0;changed+=1
    return result,changed
