"""Known straight-pit geometric support preview, independent of contact truth."""
import numpy as np
from ddevsim.chassis_contact import road_surface


def wheel_support_height(points,scene,radius=.263):
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=2 or not np.isfinite(p).all() or not np.isfinite(radius) or radius<=0:
        raise ValueError('finite N by2 wheel positions and positive radius required')
    start=scene['start_station_m']; end=start+scene['length_m']; e=scene['edge_transition_m']
    sy=np.clip(np.minimum((p[:,1]-(scene['center_y_m']-scene['width_m']/2))/e,
                         ((scene['center_y_m']+scene['width_m']/2)-p[:,1])/e),0,1)
    slope=scene['depth_m']/e*sy
    tangent=radius*slope/np.sqrt(1+slope*slope)
    edges=np.array([start,start+e,end-e,end])
    offsets=np.concatenate((np.zeros((len(p),1)),np.tile([-radius,radius],(len(p),1)),
        np.clip(edges[None,:]-p[:,0,None],-radius,radius),tangent[:,None],-tangent[:,None]),axis=1)
    candidates=np.tile(p[:,None,:],(1,offsets.shape[1],1))
    candidates[:,:,0]+=offsets
    height=road_surface(candidates,scene)[0]
    return np.max(height+np.sqrt(np.maximum(0,radius*radius-offsets*offsets))-radius,axis=1)


def exit_relief(points,scene,amplitude_n):
    """Finite-duration unloading at the exit lip, before reloading on flat road."""
    p=np.asarray(points,float)
    end=scene['start_station_m']+scene['length_m']
    progress=np.clip((p[:,0]-(end-.25))/.45,0,1)
    lateral=abs(p[:,1]-scene['center_y_m'])<scene['width_m']/2-scene['edge_transition_m']
    return -amplitude_n*np.sin(np.pi*progress)**2*lateral


def grade_torque_feedforward(points,scene,loads_n,radii_m,gain):
    """Quasi-static grade compensation from rolling-circle support geometry.

    Fz*R*sin(theta), with theta inferred from the support-height derivative.
    This is a bounded preview approximation, not native tire-moment cancellation.
    The axle sum is allocated to all available traction contacts by the caller.
    """
    points=np.asarray(points,float);delta=np.array([.002,0.])
    slope=(wheel_support_height(points+delta,scene)-wheel_support_height(points-delta,scene))/.004
    slope=np.clip(slope,-2,2)
    return gain*np.maximum(np.asarray(loads_n,float),0)*np.asarray(radii_m,float)*slope/np.sqrt(1+slope*slope)


def climb_resistance_envelope(points,scene,loads_n,radii_m,gain,lag_m,heading):
    """Bounded spatial uncertainty window for finite-patch/contact transients.

    This margin is a simulation assumption, not a measured tire footprint.
    Speed-error gating in the controller removes surplus climbing assistance.
    """
    offsets=np.linspace(0,lag_m,7)
    candidates=np.asarray(points)[:,None,:]-offsets[None,:,None]*np.asarray(heading)[None,None,:]
    loads=np.repeat(loads_n,7);radii=np.repeat(radii_m,7)
    torques=grade_torque_feedforward(candidates.reshape(-1,2),scene,loads,radii,gain).reshape(-1,7)
    return np.maximum(0,torques.max(axis=1))
