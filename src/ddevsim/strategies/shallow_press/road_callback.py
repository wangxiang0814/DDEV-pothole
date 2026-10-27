"""Unadopted contact comparison through the documented VS/STI road callback.

Signature: VS_STI_GET_ROAD_CONTACT_EXT_FUNC in vs_sti_tire.h. Its Y,X,id,
Z,dZ/dY,dZ/dX,muY,muX query is evaluated at the actual requested location.
The native public vs_get_road_contact uses one isotropic mu output; this adapter
is restricted to the straight, isotropic research scenes. No Swift API is used.
"""
import csv
import ctypes
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from .preview import wheel_support_height
from ddevsim.chassis_contact import road_surface

WHEELS=('L1','R1','L2','R2')
DOUBLE_PTR=ctypes.POINTER(ctypes.c_double)
ROAD_CALLBACK=ctypes.CFUNCTYPE(None,ctypes.c_double,ctypes.c_double,ctypes.c_int,
    DOUBLE_PTR,DOUBLE_PTR,DOUBLE_PTR,DOUBLE_PTR,DOUBLE_PTR)


def circle_planes(points,scene,radius=.263):
    """Return plane height at each hub XY and dZ/dX,dZ/dY (SI).

    Longitudinal massless rigid-circle support only; not a calibrated tire belt
    or finite footprint. Native vertical compliance and tire coefficients remain.
    """
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=2 or not np.isfinite(p).all():raise ValueError('finite N by2 hub positions required')
    epsilon=.0005
    samples=np.concatenate((p,p+[epsilon,0],p-[epsilon,0],p+[0,epsilon],p-[0,epsilon]))
    h=wheel_support_height(samples,scene,radius).reshape(5,len(p))
    gx=(h[1]-h[2])/(2*epsilon);gy=(h[3]-h[4])/(2*epsilon)
    z0=radius+h[0]-radius*np.sqrt(1+gx*gx+gy*gy)
    return np.column_stack((z0,gx,gy))


class CallbackRoad:
    def __init__(self,exports,scene,solver_dll,wrapper_dll,mode):
        if mode not in ('passthrough','circle'):raise ValueError('explicit unadopted callback mode required')
        self.mode=mode;self.scene=dict(scene);self.solver_dll=solver_dll;self.wrapper_dll=wrapper_dll
        self.index={n:i for i,n in enumerate(exports)}
        if any(a+'_'+w not in self.index for a in ('X','Y') for w in WHEELS):raise ValueError('four hub XY exports required')
        self.points=None;self.planes=None;self.time=0.;self.installed=False
        self.original=None;self.query_count=0;self.unsupported_queries=0;self.error=None
        self.rows=[];self.last_sample={}
        self.wheel_by_instance={}

    def update(self,t,exports):
        self.time=float(t)
        p=np.array([[exports[self.index['X_'+w]],exports[self.index['Y_'+w]]] for w in WHEELS],float)
        if not np.isfinite(p).all():raise ValueError('nonfinite native hub position')
        self.points=p
        if self.mode=='circle':
            # Exactly flat support outside the pit-centre domain, including
            # the finite-difference margin; no temporal/geometric decimation.
            e=.0005;start=self.scene['start_station_m'];end=start+self.scene['length_m']
            active=(p[:,0]>=start-e)&(p[:,0]<=end+e)&(abs(p[:,1]-self.scene['center_y_m'])<=self.scene['width_m']/2+e)
            self.planes=np.zeros((4,3))
            if active.any():self.planes[active]=circle_planes(p[active],self.scene)

    def install(self):
        if self.installed:return
        if self.solver_dll is None or self.wrapper_dll is None:raise ValueError('native callback libraries required')
        self.solver=ctypes.CDLL(str(self.solver_dll));self.wrapper=ctypes.CDLL(str(self.wrapper_dll))
        # vs_api.h: y,x,inst,z,dzdy,dzdx,mu. Do not pass the eight-argument
        # external callback signature to this seven-argument native function.
        self.original=self.solver.vs_get_road_contact
        self.original.argtypes=[ctypes.c_double,ctypes.c_double,ctypes.c_int,DOUBLE_PTR,DOUBLE_PTR,DOUBLE_PTR,DOUBLE_PTR]
        self.original.restype=None
        self.callback=ROAD_CALLBACK(self.query)  # retained until process exit
        setter=self.wrapper.vs_sti_get_road_contact_ext;setter.argtypes=[ROAD_CALLBACK];setter.restype=None
        setter(self.callback);self.installed=True

    def query(self,y,x,instance,z,dy,dx,muy,mux):
        self.query_count+=1
        original_valid=False
        try:
            self.original(y,x,instance,z,dy,dx,mux);muy[0]=mux[0]
            original=(float(z[0]),float(dx[0]),float(dy[0]),float(mux[0]))
            if not all(math.isfinite(v) for v in original):raise ValueError('nonfinite original road result')
            original_valid=True
            i=-1;plane=(0.,0.,0.);centre=(0.,0.)
            if self.points is not None:
                if instance not in self.wheel_by_instance:
                    distances=np.sum((self.points-[x,y])**2,axis=1);self.wheel_by_instance[instance]=int(np.argmin(distances))
                i=self.wheel_by_instance[instance];centre=self.points[i]
                if self.mode=='circle':
                    if (centre[0]-x)**2+(centre[1]-y)**2>.6**2:
                        self.unsupported_queries+=1
                    else:
                        plane=self.planes[i];z[0]=plane[0]+plane[1]*(x-centre[0])+plane[2]*(y-centre[1])
                        dx[0]=plane[1];dy[0]=plane[2]
            if self.time-self.last_sample.get(instance,-np.inf)>=.001-1e-9:
                self.last_sample[instance]=self.time
                self.rows.append(dict(command_time_s=self.time,instance=int(instance),nearest_wheel=i,
                    query_x_m=float(x),query_y_m=float(y),returned_z_m=float(z[0]),
                    returned_dzdx=float(dx[0]),returned_dzdy=float(dy[0]),mu_x=float(mux[0]),mu_y=float(muy[0]),
                    original_z_m=original[0],original_dzdx=original[1],original_dzdy=original[2],
                    hub_x_m=float(centre[0]),hub_y_m=float(centre[1]),plane_z_at_hub_m=float(plane[0]),
                    plane_dzdx=float(plane[1]),plane_dzdy=float(plane[2])))
        except Exception as exc:
            # ctypes callbacks cannot propagate exceptions through the solver.
            # Keep the full diagnostic run, mark it invalid, and return a finite
            # fallback rather than exposing undefined pointer values to C.
            self.error=repr(exc)
            if not original_valid or not all(math.isfinite(v[0]) for v in (z,dy,dx,muy,mux)):
                height,normal=road_surface(np.array([[x,y]]),self.scene)
                z[0]=height[0];dx[0]=-normal[0,0]/normal[0,2];dy[0]=-normal[0,1]/normal[0,2]
                muy[0]=mux[0]=self.scene['friction']

    def write(self,run):
        run=Path(run);path=run/'contact_callback.csv'
        with path.open('x',newline='',encoding='utf-8') as stream:
            fields=list(self.rows[0]) if self.rows else ['command_time_s']
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(self.rows)
        sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
        record=dict(mode=self.mode,installed=self.installed,query_count=self.query_count,
            sampled_queries=len(self.rows),unsupported_queries=self.unsupported_queries,error=self.error,
            interface_valid=bool(self.installed and self.query_count and not self.error and not self.unsupported_queries),
            csv_sha256=sha(path),adopted=False,physical_road_unchanged=True,
            wheel_by_instance={int(k):int(v) for k,v in self.wheel_by_instance.items()},
            assumptions=['Actual native query coordinates; latest hub state delayed one integration step',
                'Isotropic road friction delegated through public vs_get_road_contact',
                'Circle mode is a longitudinal massless support-plane approximation, not MF-Swift or a calibrated tire'])
        (run/'contact_callback.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
        return dict(**record,summary_sha256=sha(run/'contact_callback.json'))
