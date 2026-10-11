"""Shared diagnostic geometry and optional uncalibrated compliant contact.

Coordinates are SI, +X forward, +Y left, +Z up. Geometry uses the physical
bilinear pit map, never the visualizer offsets. Forces require a NEW native
plant with reference-point imports; they must not be fed into suspension ports.
"""
from __future__ import annotations

import re
import numpy as np


def rotation_matrix(attitude_deg):
    roll,pitch,yaw=np.deg2rad(np.asarray(attitude_deg,dtype=float))
    cr,sr=np.cos(roll),np.sin(roll)
    cp,sp=np.cos(pitch),np.sin(pitch)
    cy,sy=np.cos(yaw),np.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],
                     [-sp,cp*sr,cp*cr]])


def transform_points(points, origin_m, attitude_deg):
    p=np.asarray(points,dtype=float)
    o=np.asarray(origin_m,dtype=float)
    if p.ndim!=2 or p.shape[1]!=3 or o.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(o).all():
        raise ValueError('finite N x 3 points and origin required')
    if not np.isfinite(attitude_deg).all():
        raise ValueError('finite attitude required')
    return p @ rotation_matrix(attitude_deg).T + o


def road_surface(points, scene):
    """Height and upward unit normal from the generated straight bilinear map."""
    p=np.asarray(points,dtype=float)
    e=float(scene['edge_transition_m']);d=float(scene['depth_m'])
    if not 0<2*e<min(scene['length_m'],scene['width_m']) or d<0:
        raise ValueError('invalid physical pit geometry')
    def ramp(x,lo,hi):
        a=(x-lo)/e;b=(hi-x)/e
        weight=np.clip(np.minimum(a,b),0.,1.)
        slope=np.where((a>0)&(a<1),1/e,np.where((b>0)&(b<1),-1/e,0.))
        return weight,slope
    sx,dx=ramp(p[...,0],scene['start_station_m'],scene['start_station_m']+scene['length_m'])
    sy,dy=ramp(p[...,1],scene['center_y_m']-scene['width_m']/2,scene['center_y_m']+scene['width_m']/2)
    z=-d*sx*sy
    n=np.stack((d*dx*sy,d*sx*dy,np.ones_like(z)),axis=-1)
    n/=np.linalg.norm(n,axis=-1,keepdims=True)
    return z,n


def contact_forces(world_points, world_velocity, scene, stiffness_n_m, damping_n_s_m, friction, slip_regularization_m_s):
    """Unilateral penalty and dissipative regularized Coulomb friction per point.

    Signed normal penetration is the vertical gap projected onto the local
    normal. This is a compliant surface-point model, not rigid solid collision.
    """
    if not (stiffness_n_m>0 and damping_n_s_m>=0 and 0<=friction<=2 and slip_regularization_m_s>0):
        raise ValueError('invalid contact constitutive parameters')
    p=np.asarray(world_points,dtype=float);v=np.asarray(world_velocity,dtype=float)
    if p.shape!=v.shape or not np.isfinite(p).all() or not np.isfinite(v).all():
        raise ValueError('finite matching point and velocity arrays required')
    height,n=road_surface(p,scene)
    gap=p[:,2]-height
    vn=np.sum(v*n,axis=1)
    normal=np.where(gap<=0,np.maximum(0.,-stiffness_n_m*gap*n[:,2]-damping_n_s_m*vn),0.)
    vt=v-vn[:,None]*n
    speed=np.linalg.norm(vt,axis=1)
    friction_force=-friction*normal[:,None]*vt/np.sqrt(speed[:,None]**2+slip_regularization_m_s**2)
    return normal[:,None]*n+friction_force,gap


def sample_plate(bounds, z_m, spacing_m):
    if not (0<spacing_m<=.2 and bounds[0]<bounds[1] and bounds[2]<bounds[3]):
        raise ValueError('valid underside bounds and sampling spacing required')
    x=np.linspace(bounds[0],bounds[1],int(np.ceil((bounds[1]-bounds[0])/spacing_m))+1)
    y=np.linspace(bounds[2],bounds[3],int(np.ceil((bounds[3]-bounds[2])/spacing_m))+1)
    xx,yy=np.meshgrid(x,y,indexing='ij')
    return np.column_stack((xx.ravel(),yy.ravel(),np.full(xx.size,z_m)))


def plate_clearance(corners, scene):
    """Exact minimum vertical gap of a planar rectangle to the bilinear pit.

    Each map cell is bilinear: gap has no strict interior minimum (its Hessian
    is indefinite or zero). Check cell vertices and quadratic edge stationary
    points. This avoids missing a pit lip between underside sampling points.
    The underside must project to a nondegenerate parallelogram in XY.
    """
    c=np.asarray(corners,dtype=float)
    if c.ndim!=3 or c.shape[1:]!=(4,3) or not np.isfinite(c).all():
        raise ValueError('finite batch of four ordered rectangle corners required')
    x0=scene['start_station_m'];x1=x0+scene['length_m'];e=scene['edge_transition_m']
    y0=scene['center_y_m']-scene['width_m']/2;y1=y0+scene['width_m']
    xknots=np.array([x0,x0+e,x1-e,x1]);yknots=np.array([y0,y0+e,y1-e,y1])
    candidates=[];valid=[]
    for j in range(4):
        a=c[:,j];delta=c[:,(j+1)%4]-a
        cuts=[np.zeros((len(c),1)),np.ones((len(c),1))]
        for axis,knots in [(0,xknots),(1,yknots)]:
            ratios=np.zeros((len(c),4))
            np.divide(knots[None,:]-a[:,axis,None],delta[:,axis,None],out=ratios,
                      where=abs(delta[:,axis,None])>1e-12)
            cuts.append(np.clip(ratios,0,1))
        cut=np.sort(np.concatenate(cuts,axis=1),axis=1)
        lo=cut[:,:-1];hi=cut[:,1:];mid=(lo+hi)/2
        p0=a[:,None,:]+lo[:,:,None]*delta[:,None,:]
        p1=a[:,None,:]+hi[:,:,None]*delta[:,None,:]
        pm=a[:,None,:]+mid[:,:,None]*delta[:,None,:]
        f0=p0[:,:,2]-road_surface(p0,scene)[0]
        f1=p1[:,:,2]-road_surface(p1,scene)[0]
        fm=pm[:,:,2]-road_surface(pm,scene)[0]
        qa=2*(f1+f0-2*fm);qb=f1-f0-qa
        tau=np.zeros_like(qa)
        np.divide(-qb,2*qa,out=tau,where=qa>1e-12)
        tau=np.clip(tau,0,1)
        root=lo+tau*(hi-lo)
        points=np.concatenate((p0,p1,a[:,None,:]+root[:,:,None]*delta[:,None,:]),axis=1)
        candidates.append(points);valid.append(np.ones(points.shape[:2],dtype=bool))
    # Internal map-grid intersections can be the minimum on a cell edge.
    gx,gy=np.meshgrid(xknots,yknots,indexing='ij');xy=np.column_stack((gx.ravel(),gy.ravel()))
    a=c[:,0];u=c[:,1]-a;v=c[:,3]-a
    det=u[:,0]*v[:,1]-u[:,1]*v[:,0]
    if np.any(abs(det)<1e-8):
        raise ValueError('vertical underside projection is degenerate')
    q=xy[None,:,:]-a[:,None,:2]
    s=(q[:,:,0]*v[:,None,1]-q[:,:,1]*v[:,None,0])/det[:,None]
    t=(u[:,None,0]*q[:,:,1]-u[:,None,1]*q[:,:,0])/det[:,None]
    grid=a[:,None,:]+s[:,:,None]*u[:,None,:]+t[:,:,None]*v[:,None,:]
    candidates.append(grid);valid.append((s>=0)&(s<=1)&(t>=0)&(t<=1))
    points=np.concatenate(candidates,axis=1)
    gaps=points[:,:,2]-road_surface(points,scene)[0]
    gaps=np.where(np.concatenate(valid,axis=1),gaps,np.inf)
    i=np.argmin(gaps,axis=1)
    return gaps[np.arange(len(c)),i],points[np.arange(len(c)),i]


def instrument_reference_points(text, simfile, points):
    """Append independent inertial forces/position exports to a fresh model.

    TruckSim 2019 native example Output/Points supplies the installation syntax.
    Body parameter X/Y/Z_RP is mm; native X/Y/Z_RP1 outputs are metres.
    """
    if 'DEFINE_REFERENCE_POINTS' in text:
        raise ValueError('model already has reference points')
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not 1<=len(points)<=99 or not np.isfinite(points).all():
        raise ValueError('finite reference points required')
    imports=re.findall(r'(?m)^IMPORT\s+(\S+)',text)
    exports=re.findall(r'(?m)^EXPORT\s+(\S+)',text)
    for kind,names in [('IMP',imports),('EXP',exports)]:
        m=re.search(r'(?m)^PORTS_'+kind+r'\s+(\d+)',simfile)
        if m is None or int(m.group(1))!=len(names):
            raise ValueError('original port count mismatch')
    commands=['IREF_PT = 1','DEFINE_REFERENCE_POINTS %d'%len(points)]
    added_imp=[];added_exp=[]
    for i,(x,y,z) in enumerate(points,1):
        commands += ['OPT_RP_BODY_TYPE(%d) 1'%i,'OPT_RP_BODY_ID(%d) 1'%i,'OPT_RP_FORCE_N(%d) 1'%i]
        commands += ['%s_RP(%d) %.12g'%(axis,i,value*1000) for axis,value in zip('XYZ',(x,y,z))]
        added_imp += ['IMP_F%s_RP_%d'%(axis,i) for axis in 'XYZ']
        added_exp += ['%s_RP%d'%(axis,i) for axis in 'XYZ']
    pos=text.index('IMPORT ')
    text=text[:pos]+'\n'.join(commands)+'\n'+text[pos:]
    end=re.search(r'(?m)^END\s*\Z',text)
    if end is None:
        raise ValueError('missing terminal END')
    text=text[:end.start()]+'\n'.join(['IMPORT '+n+' Replace 0.0' for n in added_imp]+['EXPORT '+n for n in added_exp])+'\n'+text[end.start():]
    for kind,names in [('IMP',imports+added_imp),('EXP',exports+added_exp)]:
        simfile=re.sub(r'(?m)^PORTS_'+kind+r'\s+\d+', 'PORTS_'+kind+' '+str(len(names)),simfile)
    return text,simfile,imports+added_imp,exports+added_exp
