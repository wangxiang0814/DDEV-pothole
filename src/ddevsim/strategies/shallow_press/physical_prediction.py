"""Native affine SI predictor and an explicit unilateral tyre approximation.

The clamp feeds missing tensile load back into the differential equations via
the native height-input derivative. It does not model contact-patch migration.
"""
import numpy as np
from scipy.linalg import expm
from ddevsim.chassis_contact import road_surface


def preview_physical_road(start_xy, *, speed_mps, yaw_rad, dt, steps, scene):
    """Causal constant-velocity map preview at the current contact-patch pose."""
    xy=np.asarray(start_xy,float)
    if (xy.shape!=(4,2) or not np.isfinite(xy).all()
            or not np.isfinite([speed_mps,yaw_rad,dt]).all() or dt<=0 or steps<1):
        raise ValueError('finite current contact positions and positive horizon required')
    points=np.tile(xy,(steps,1,1))
    velocity=speed_mps*np.array([np.cos(yaw_rad),np.sin(yaw_rad)])
    points+=np.arange(steps)[:,None,None]*dt*velocity[None,None,:]
    height,normal=road_surface(points,scene)
    return np.concatenate((height,-normal[...,0]/normal[...,2],
                           -normal[...,1]/normal[...,2]),axis=1)


def physical_rollout(model, state_delta, inputs, dt, *, unilateral=False, max_step_s=.001):
    return physical_rollout_batch(model,np.asarray(state_delta,float)[None,:],
        np.asarray(inputs,float)[None,:,:],dt,unilateral=unilateral,max_step_s=max_step_s)[0]


def physical_rollout_batch(model, state_delta, inputs, dt, *, unilateral=False, max_step_s=.001):
    """Independent horizons batched for speed; no coupling across initial states."""
    A,B,C,D = [np.asarray(model[k],float) for k in ('A','B','C','D')]
    x=np.asarray(state_delta,float).copy()
    u=np.asarray(inputs,float)
    f0=np.asarray(model['f0'],float)
    y0=np.asarray(model['y0'],float)
    n=x.shape[-1]
    if (x.ndim!=2 or A.shape!=(n,n) or B.ndim!=2 or B.shape[0]!=n or C.ndim!=2 or C.shape[1]!=n
            or D.shape!=(C.shape[0],B.shape[1]) or u.ndim!=3 or u.shape[0]!=x.shape[0] or u.shape[2]!=B.shape[1]
            or f0.shape not in ((n,),(x.shape[0],n)) or y0.shape not in ((C.shape[0],),(x.shape[0],C.shape[0])) or dt<=0 or max_step_s<=0
            or not np.isfinite([dt,max_step_s]).all()
            or not all(np.isfinite(v).all() for v in (A,B,C,D,x,u,f0,y0))):
        raise ValueError('finite consistent affine state-space model required')
    if not unilateral:
        aug=np.zeros((2*n,)*2)
        aug[:n,:n]=A; aug[:n,n:]=np.eye(n)
        zoh=expm(aug*dt)
        result=[]
        for tick in range(u.shape[1]):
            command=u[:,tick]
            x=x@zoh[:n,:n].T+(command@B.T+f0)@zoh[:n,n:].T
            output=y0+x@C.T+command@D.T
            if not np.isfinite(output).all(): raise ValueError('nonfinite physical rollout')
            result.append(output)
        return np.stack(result,axis=1)
    loads=np.asarray(model['load_rows'],int)
    heights=np.asarray(model['height_cols'],int)
    if len(loads)!=len(heights) or not len(loads):
        raise ValueError('paired load rows and road-height columns required')
    stiffness=D[np.ix_(loads,heights)]
    if np.linalg.matrix_rank(stiffness)!=len(loads):
        raise ValueError('invertible native tyre-height sensitivity required')
    inverse_stiffness=np.linalg.inv(stiffness)
    correction=B[:,heights]@inverse_stiffness
    output_correction=D[:,heights]@inverse_stiffness
    origin=np.asarray(model.get('contact_correction_origin',np.zeros(len(loads))),float)
    if origin.shape not in ((len(loads),),(x.shape[0],len(loads))) or not np.isfinite(origin).all():
        raise ValueError('finite contact correction origin required')
    steps=int(np.ceil(dt/max_step_s)); h=dt/steps
    result=[]
    for tick in range(u.shape[1]):
        command=u[:,tick]
        constant=command@B.T+f0
        output_bias=y0+command@D.T
        def derivative(state):
            predicted_load=(output_bias+state@C.T)[:,loads]
            return state@A.T+constant+(np.maximum(-predicted_load,0)-origin)@correction.T
        for _ in range(steps):
            k1=derivative(x); k2=derivative(x+.5*h*k1)
            k3=derivative(x+.5*h*k2); k4=derivative(x+h*k3)
            x += h/6*(k1+2*k2+2*k3+k4)
        output=output_bias+x@C.T
        # Force clipping changes direct-feedthrough accelerations too. For an
        # anchored current drift/output, remove the already-present correction.
        load_correction=np.maximum(-output[:,loads],0)-origin
        corrected=output+load_correction@output_correction.T
        nonloads=np.setdiff1d(np.arange(C.shape[0]),loads)
        output[:,nonloads]=corrected[:,nonloads]
        output[:,loads]=np.maximum(output[:,loads],0)
        if not np.isfinite(output).all():
            raise ValueError('nonfinite physical rollout')
        result.append(output)
    return np.stack(result,axis=1)
