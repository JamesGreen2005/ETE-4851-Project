"""Independent metric motion observations. No filter feedback is used as input."""
import math
import numpy as np


def rigid_fit(source, target):
    a=np.asarray(source,dtype=float);b=np.asarray(target,dtype=float)
    if a.shape!=b.shape or a.ndim!=2 or a.shape[1]!=2 or len(a)<12:
        raise ValueError('At least 12 paired planar points required')
    if not np.isfinite(a).all() or not np.isfinite(b).all():raise ValueError('Nonfinite points')
    ac=a-a.mean(axis=0);bc=b-b.mean(axis=0)
    # A single wall / line does not constrain both translation directions.
    if np.linalg.eigvalsh(ac.T@ac/len(a))[0]<.0025:raise ValueError('Degenerate geometry')
    u,_,vt=np.linalg.svd(ac.T@bc);r=vt.T@u.T
    if np.linalg.det(r)<0:vt[-1]*=-1;r=vt.T@u.T
    t=b.mean(axis=0)-r@a.mean(axis=0)
    residual=np.linalg.norm(a@r.T+t-b,axis=1)
    return r,t,float(np.sqrt(np.mean(residual**2)))


def scan_motion(current, previous, iterations=12):
    """Trimmed point-to-point ICP: current base-frame scan into previous base."""
    a=np.asarray(current,float);b=np.asarray(previous,float)
    if min(len(a),len(b))<40:raise ValueError('Too few scan returns')
    r=np.eye(2);t=np.zeros(2);rmse=float('inf');count=0
    for _ in range(iterations):
        moved=a@r.T+t
        d2=np.sum((moved[:,None,:]-b[None,:,:])**2,axis=2)
        indices=np.argmin(d2,axis=1);errors=np.sqrt(d2[np.arange(len(a)),indices])
        keep=errors<min(.20,float(np.quantile(errors,.75)))
        # Include exact correspondences in stationary scans.
        keep=errors<=min(.20,max(.00001,float(np.quantile(errors,.75))))
        count=int(keep.sum())
        if count<40 or count/len(a)<.55:raise ValueError('Insufficient scan overlap')
        dr,dt,rmse=rigid_fit(moved[keep],b[indices[keep]])
        r=dr@r;t=dr@t+dt
        if np.linalg.norm(dt)<.0001 and abs(math.atan2(dr[1,0],dr[0,0]))<.0001:break
    if rmse>.04:raise ValueError('Scan residual too high')
    return r,t,max(.0004,rmse**2),count/len(a)


def motion_twist(r,t,dt,max_speed=1.):
    if not .04<=dt<=.5:raise ValueError('Invalid observation interval')
    yaw=math.atan2(r[1,0],r[0,0]);velocity=r.T@t/dt
    if np.linalg.norm(velocity)>max_speed or abs(yaw/dt)>3.:raise ValueError('Motion outlier')
    return float(velocity[0]),float(velocity[1]),yaw/dt


def wheel_variance(sensors, reference_current=None):
    """Load is a confidence cue, never a fabricated position observation."""
    currents=[abs(m['current_a']) for m in sensors.get('motors',[])
              if m.get('current_status')=='ok' and m.get('current_a') is not None]
    if not sensors.get('fresh') or len(currents)<2:return .16,'Current incomplete; conservative wheel confidence'
    imbalance=abs(currents[0]-currents[1])/max(.1,sum(currents))
    factor=1+8*imbalance**2
    if reference_current is not None:factor+=min(20.,(max(currents)/reference_current)**2)
    return .04*factor,'Load-weighted wheel confidence'


def combine_wheel_speeds(primary,secondary):
    """Two calibrated longitudinal estimates; disagreement increases variance."""
    if secondary is None:return primary,0.
    if not math.isfinite(primary+secondary):raise ValueError('Nonfinite wheel speed')
    return (primary+secondary)/2.,(primary-secondary)**2
