"""Parameterised underside clearance for existing native histories; no force retrofit."""
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from ddevsim.chassis_contact import rotation_matrix,plate_clearance
from analyze_speed_depth_baseline import read_channels


def resolve_geometry(config, clearance):
    run=ROOT/config['calibration_run'];v=read_channels(run/'native_5ms.csv')
    mask=(v['time_s']>=config['static_window_s'][0])&(v['time_s']<config['static_window_s'][1])
    attitude=[float(v['exp_'+n][mask].mean()) for n in ['Roll_E','Pitch','Yaw']]
    origin=[float(v['exp_'+n][mask].mean()) for n in ['Xo','Yo','Zo']]
    r=rotation_matrix(attitude);x0,x1,y0,y1=config['underside_bounds_m']
    xy=np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])
    z=(clearance-origin[2]-np.min(xy@r[2,:2]))/r[2,2]
    return np.column_stack((xy,np.full(4,z))),dict(local_z_m=float(z),calibration_origin_m=origin,
                calibration_attitude_deg=attitude,requested_static_min_clearance_m=clearance,
                calibration_csv_sha256=hashlib.sha256((run/'native_5ms.csv').read_bytes()).hexdigest())


def world_corners(v,points,indices):
    origins=np.column_stack([v['exp_'+n][indices] for n in ['Xo','Yo','Zo']])
    attitudes=np.column_stack([v['exp_'+n][indices] for n in ['Roll_E','Pitch','Yaw']])
    matrices=np.stack([rotation_matrix(a) for a in attitudes])
    return np.einsum('nij,kj->nki',matrices,points)+origins[:,None,:]


def analyse(run,config,clearance):
    scene=json.loads((run/'model/scenario.json').read_text(encoding='utf-8'))
    csv_path=run/'native_5ms.csv'
    if not csv_path.exists():
        csv_path=run/'native_1ms.csv'
    v=read_channels(csv_path);points,geometry=resolve_geometry(config,clearance)
    manifest=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
    force_extension=manifest.get('diagnostic')=='CHASSIS_CONTACT_EXTENSION_V1' and not manifest.get('diagnostic_options',{}).get('contact_off',True)
    time=v['time_s'];dt=float(np.median(np.diff(time)))
    gaps=[];locations=[]
    for start in range(0,len(time),512):
        indices=np.arange(start,min(start+512,len(time)))
        gap,p=plate_clearance(world_corners(v,points,indices),scene)
        gaps.append(gap);locations.append(p)
    gap=np.concatenate(gaps);p=np.concatenate(locations);active=time>=5.
    i=np.flatnonzero(active)[np.argmin(gap[active])]
    hits=np.flatnonzero(active&(gap<=0))
    entry=(gap<=0)&active;starts=np.flatnonzero(entry&~np.r_[False,entry[:-1]])
    ends=np.flatnonzero(entry&~np.r_[entry[1:],False])
    # Historical trajectory lacks chassis forces. Only the prefix to first
    # geometrical intersection can be interpreted as an unobstructed motion.
    result=dict(run_path=str(run.relative_to(ROOT)),case=scene['case_id'],depth_m=scene['depth_m'],
                nominal_static_clearance_m=clearance,geometry=geometry,
                status='ASSUMED_GEOMETRIC_INTERSECTION' if len(hits) else 'NO_INTERSECTION_FOR_ASSUMED_PLANE',
                first_intersection_s=float(time[hits[0]]) if len(hits) else None,
                min_gap_m=float(gap[i]),minimum_time_s=float(time[i]),minimum_location_m=p[i].tolist(),
                penetration_duration_s=float(entry.sum()*dt),
                events=[dict(start_s=float(time[a]),end_s=float(time[b]),duration_s=float(time[b]-time[a]+dt)) for a,b in zip(starts,ends)],
                interpretation_after_first_intersection=('COMPLIANT_POINT_MODEL_TRAJECTORY' if force_extension and
                    abs(clearance-manifest['diagnostic_options']['clearance_m'])<1e-9 else 'COUNTERFACTUAL_FOR_THIS_ASSUMED_GEOMETRY'),
                physical_contact_force='ASSUMED_POINT_FORCE_EXTENSION_PRESENT' if force_extension else 'NOT_PRESENT_IN_HISTORICAL_MODEL',
                source_contact_clearance_m=manifest.get('diagnostic_options',{}).get('clearance_m') if force_extension else None,
                csv_sha256=hashlib.sha256(csv_path.read_bytes()).hexdigest())
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'configs/diagnostics/chassis_contact_v1.json')
    p.add_argument('--run',type=Path)
    p.add_argument('--output',type=Path,default=ROOT/'evidence/chassis_contact_20261011')
    a=p.parse_args();config=json.loads(a.config.read_text(encoding='utf-8'))
    if a.output.exists():
        raise FileExistsError('analysis output already exists; choose another --output')
    a.output.mkdir(parents=True)
    runs=[a.run.resolve()] if a.run else [ROOT/r['run_path'] for r in json.loads((ROOT/'evidence/speed_depth_baseline_20261011/summary.json').read_text(encoding='utf-8'))['results']]
    results=[]
    for run in runs:
        for h in config['static_min_clearance_sweep_m']:
            r=analyse(run,config,h);results.append(r)
            print(run.name,h,r['status'],round(r['min_gap_m'],4),flush=True)
    report=dict(scope='PARAMETRIC_HYPOTHESIS_NOT_REAL_CHASSIS_CERTIFICATION',config=config,
                config_sha256=hashlib.sha256(a.config.read_bytes()).hexdigest(),
                metric_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                geometry_code_sha256=hashlib.sha256((ROOT/'src/ddevsim/chassis_contact.py').read_bytes()).hexdigest(),
                algorithm='Exact planar rectangle minimum over bilinear physical map; 1 ms sampled time',
                results=results)
    (a.output/'clearance_summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    lines=['# 历史工况底盘净空敏感性', '', '假定簧载矩形底面；静态离地间隙10/15/20/25/30 cm。非真实底盘验收。', '',
           '| 坑型 | 运行名称 | 假定静态间隙 cm | 最小净空 mm | 首次几何相交 s |', '|---|---|---:|---:|---|']
    for r in results:
        lines.append('| %s | `%s` | %.0f | %.1f | %s |'%(r['case'],Path(r['run_path']).name,r['nominal_static_clearance_m']*100,r['min_gap_m']*1000,'无' if r['first_intersection_s'] is None else '%.4f'%r['first_intersection_s']))
    lines+=['','出现首次几何相交后，旧轨迹没有底盘反力，后续穿透量与持续时间只用于定位问题，不能当作真实撞击压缩或冲击力。',
            '矩形底面没有桥壳、排气管及其他底部件；无相交仅适用于该假定包络。完整参数、位置与事件见JSON。','']
    (a.output/'clearance_summary.md').write_text('\n'.join(lines),encoding='utf-8')


if __name__=='__main__':
    main()
