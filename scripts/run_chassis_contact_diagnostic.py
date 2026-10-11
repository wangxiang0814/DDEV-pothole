"""Isolated TruckSim diagnostic extension: assumed underside contact forces."""
import argparse, json, sys, hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from ddevsim.chassis_contact import instrument_reference_points,contact_forces,transform_points,road_surface
from ddevsim.runtime.run_workspace import claim_run,write_run_manifest,sha256_file
from ddevsim.scenarios.pit import load_case
from ddevsim.scenarios.builder import prepare_case
from ddevsim.cosim import run_stepwise
from run_shallow_depth_baseline import Baseline,validate_native_result
from analyze_chassis_clearance import resolve_geometry
from analyze_speed_depth_baseline import read_channels


class ContactDiagnostic:
    def __init__(self,names,points,scene,config,speed,enabled,probe_force):
        self.index={n:i for i,n in enumerate(names)}
        self.points=points;self.scene=scene;self.config=config;self.enabled=enabled
        self.base=Baseline(names,speed_kph=speed,steering_feedback=True,observe_after_attitude_limit=True)
        self.base.end_station_m=scene['start_station_m']+scene['length_m']+.263+.8
        self.previous=None;self.previous_time=None
        self.probe_force=probe_force;self.impulse=np.zeros(3)
        self.contact_steps=0;self.peak_force=0.;self.penetration_peak=0.
        self.first_contact_s=None;self.normal_impulse_ns=0.;self.friction_work_j=0.
        self.obstruction_s=0.;self.obstruction_longest_s=0.;self.normal_peak_n=0.

    def __call__(self,t,values):
        x=lambda n: float(values[self.index[n]])
        points=np.array([[x('%s_RP%d'%(axis,i)) for axis in 'XYZ'] for i in range(1,len(self.points)+1)])
        dt=.0005 if self.previous_time is None else t-self.previous_time
        v=np.zeros_like(points) if self.previous is None else (points-self.previous)/dt
        self.previous=points;self.previous_time=t
        force,gap=contact_forces(points,v,self.scene,
                self.config['total_stiffness_n_m']/len(points),self.config['total_damping_n_s_m']/len(points),
                self.config['friction'],self.config['slip_regularization_m_s'])
        if not self.enabled:
            force[:]=0.
        if self.probe_force and 5.<=t<5.1:
            force[:,2]+=self.probe_force/len(points)
        self.impulse+=force.sum(axis=0)*dt
        self.peak_force=max(self.peak_force,float(np.linalg.norm(force.sum(axis=0))))
        self.penetration_peak=max(self.penetration_peak,float(max(0,-gap.min())))
        self.contact_steps+=int(np.any(np.linalg.norm(force,axis=1)>1.))
        _,normal=road_surface(points,self.scene)
        fn=np.sum(force*normal,axis=1)
        tangent=force-fn[:,None]*normal
        self.normal_impulse_ns+=float(fn.sum())*dt
        self.friction_work_j+=float(np.sum(tangent*v))*dt
        self.normal_peak_n=max(self.normal_peak_n,float(fn.sum()))
        if np.any(fn>1.) and self.first_contact_s is None:
            self.first_contact_s=t
        command=self.base(t,values)
        obstructed=(abs(x('Vx'))<.5 and fn.sum()>1000 and command[0]>0
                    and x('X_R1')>=self.scene['start_station_m']-.263
                    and min(x('X_L2'),x('X_R2'))<=self.scene['start_station_m']+self.scene['length_m']+.263)
        self.obstruction_s=self.obstruction_s+dt if obstructed else 0.
        self.obstruction_longest_s=max(self.obstruction_longest_s,self.obstruction_s)
        return command+tuple(force.ravel())


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=['right_track','wide'],required=True)
    p.add_argument('--depth-m',type=float,required=True)
    p.add_argument('--speed-kph',type=float,default=10.)
    p.add_argument('--clearance-m',type=float,default=.15)
    p.add_argument('--run-id',required=True)
    p.add_argument('--config',type=Path,default=ROOT/'configs/diagnostics/chassis_contact_v1.json')
    p.add_argument('--contact-off',action='store_true')
    p.add_argument('--duration-s',type=float,default=30.)
    p.add_argument('--force-probe-n',type=float,default=0.)
    p.add_argument('--grid-shape',type=int,nargs=2,metavar=('NX','NY'))
    p.add_argument('--stiffness-scale',type=float,default=1.)
    a=p.parse_args()
    if not (0<a.clearance_m<=.5 and 0<=a.speed_kph<=12 and 6<=a.duration_s<=120 and abs(a.force_probe_n)<=2000 and .1<=a.stiffness_scale<=20):
        p.error('invalid diagnostic limits')
    config=json.loads(a.config.read_text(encoding='utf-8'))
    config['total_stiffness_n_m']*=a.stiffness_scale
    config['total_damping_n_s_m']*=np.sqrt(a.stiffness_scale)
    if a.grid_shape:
        if min(a.grid_shape)<2 or np.prod(a.grid_shape)>99:
            p.error('contact grid requires >=2 points per axis and <=99 total')
        config['runtime_grid_shape']=a.grid_shape
    corners,calibration=resolve_geometry(config,a.clearance_m)
    nx,ny=config['runtime_grid_shape'];x0,x1,y0,y1=config['underside_bounds_m']
    xx,yy=np.meshgrid(np.linspace(x0,x1,nx),np.linspace(y0,y1,ny),indexing='ij')
    points=np.column_stack((xx.ravel(),yy.ravel(),np.full(xx.size,corners[0,2])))
    case=load_case(ROOT/('scenarios/shallow_press/'+a.case+'.json'),depth_m=a.depth_m,
                   length_m=.8,width_m=.9 if a.case=='right_track' else 1.8,
                   approach_distance_m=12.,stop_s=a.duration_s,friction=.7)
    run=claim_run(ROOT/'runs','shallow_press',a.case,a.run_id)
    prepare_case(case,run)
    model=run/'model';par=model/'run_all.par';sim=model/'simfile.sim'
    text,spec,imports,exports=instrument_reference_points(par.read_text(encoding='utf-8'),sim.read_text(encoding='utf-8'),points)
    par.write_text(text,encoding='utf-8');sim.write_text(spec,encoding='utf-8')
    scene=json.loads((model/'scenario.json').read_text(encoding='utf-8'))
    manifest=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
    manifest.update(diagnostic='CHASSIS_CONTACT_EXTENSION_V1',task_status='UNASSESSED',acceptance_pass=False,
                    import_names=imports,export_names=exports,diagnostic_options=vars(a),
                    contact_config=config,geometry_calibration=calibration,reference_points_m=points.tolist(),
                    base_vehicle_sha256=manifest['vehicle']['vehicle_sha256'],
                    extended_model_sha256=sha256_file(par),run_status='RUNNING')
    manifest['diagnostic_code_sha256']=sha256_file(Path(__file__))
    manifest['geometry_code_sha256']=sha256_file(ROOT/'src/ddevsim/chassis_contact.py')
    manifest['diagnostic_options']={k:str(v) if isinstance(v,Path) else v for k,v in vars(a).items()}
    manifest['generated_assets'].update({'model/run_all.par':sha256_file(par),'model/simfile.sim':sha256_file(sim)})
    write_run_manifest(run,manifest)
    control=ContactDiagnostic(exports,points,scene,config,a.speed_kph,not a.contact_off,a.force_probe_n)
    try:
        native=run_stepwise(sim,control,run/'native_1ms.csv',imports,exports,log_decimation=2)
        validate_native_result(native)
        v=read_channels(run/'native_1ms.csv');time=v['time_s']
        # Validate full trajectory, not just a flat-road initial frame.
        error=0.
        for i in range(0,len(time),20):
            actual=np.array([[v['exp_%s_RP%d'%(axis,j)][i] for axis in 'XYZ'] for j in range(1,len(points)+1)])
            expected=transform_points(points,[v['exp_'+n][i] for n in ['Xo','Yo','Zo']],
                                       [v['exp_'+n][i] for n in ['Roll_E','Pitch','Yaw']])
            error=max(error,float(np.linalg.norm(actual-expected,axis=1).max()))
        if error>.001:
            raise ValueError('native reference-frame mismatch: %.6g m'%error)
        rear=np.minimum(v['exp_X_L2'],v['exp_X_R2']) if a.case=='wide' else v['exp_X_R2']
        cleared=np.flatnonzero(rear>scene['start_station_m']+scene['length_m']+.263)
        result=dict(scope='UNCALIBRATED_PARAMETRIC_CONTACT_DIAGNOSTIC',native=native,
                    geometry_status=config['geometry_status'],constitutive_status=config['contact_model_status'],
                    native_reference_frame_error_m=error,reference_point_count=len(points),
                    contact_enabled=not a.contact_off,force_probe_n=a.force_probe_n,
                    normal_and_friction_forces_applied_to_sprung_reference_points=not a.contact_off,
                    independent_force_probe_enabled=bool(a.force_probe_n),
                    impulse_world_ns=control.impulse.tolist(),total_force_peak_n=control.peak_force,
                    maximum_point_penetration_m=control.penetration_peak,
                    contact_force_duration_s=control.contact_steps*.0005,
                    first_applied_contact_force_s=control.first_contact_s,
                    normal_impulse_ns=control.normal_impulse_ns,normal_force_peak_n=control.normal_peak_n,
                    friction_work_j=control.friction_work_j,
                    contact_obstruction_longest_s=control.obstruction_longest_s,
                    obstruction_candidate=control.obstruction_longest_s>=1.,
                    rear_cleared=bool(len(cleared)),rear_exit_s=float(time[cleared[0]]) if len(cleared) else None,
                    roll_peak_deg=float(abs(v['exp_Roll_E']).max()),pitch_peak_deg=float(abs(v['exp_Pitch']).max()),
                    lateral_peak_m=float(abs(v['exp_Yo']-v['exp_Yo'][0]).max()),
                    terminal_speed_kph=float(v['exp_Vx'][-1]),
                    outcome='REAR_CLEARED' if len(cleared) else 'DID_NOT_CLEAR_WITHIN_DURATION',
                    acceptance_pass=False,task_status='UNASSESSED')
        (run/'contact_diagnostic.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        manifest.update(run_status='DIAGNOSTIC_COMPLETED',result=result)
        write_run_manifest(run,manifest)
        print(json.dumps(result,indent=2))
    except Exception as exc:
        manifest.update(run_status='DIAGNOSTIC_FAILED',error=str(exc));write_run_manifest(run,manifest)
        raise


if __name__=='__main__':
    main()
