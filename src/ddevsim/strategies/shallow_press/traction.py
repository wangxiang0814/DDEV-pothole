"""Classical side-balanced load-aware allocation and low-speed slip feedback."""
import numpy as np


def allocate_torque(total,loads,slip,yaw_error,yaw_rate,friction,radii,slip_gain=180.):
    loads=np.maximum(np.asarray(loads,float),0)
    radii=np.asarray(radii,float); slip=np.asarray(slip,float)
    if loads.shape!=(4,) or radii.shape!=(4,) or slip.shape!=(4,) or not np.isfinite(np.r_[total,loads,slip,yaw_error,yaw_rate,friction,radii]).all():
        raise ValueError('finite four-wheel allocation required')
    capacity=np.where(loads>150,np.minimum(500,.65*max(0,friction)*loads*radii),0)
    moment=np.clip(-80*yaw_error-25*yaw_rate,-abs(total)*.15,abs(total)*.15)
    requested=((total-moment)/2,(total+moment)/2)
    result=np.zeros(4)
    for indices,target in zip(([0,2],[1,3]),requested):
        indices=np.asarray(indices); weight=loads[indices]*(capacity[indices]>0)
        if weight.sum()<=0: continue
        remaining=target
        for _ in range(3):
            free=abs(result[indices])<capacity[indices]-1e-8
            w=weight*free
            if w.sum()<=0: break
            before=result[indices].copy()
            result[indices]=np.clip(before+remaining*w/w.sum(),-capacity[indices],capacity[indices])
            remaining-=sum(result[indices]-before)
    return np.clip(result-np.where(capacity>0,slip_gain*slip,0),-capacity,capacity)
