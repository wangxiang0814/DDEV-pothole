"""Phase-resolved passive baseline diagnostics; never certifies vehicle safety."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from analyze_speed_depth_baseline import ROOT, read_channels
from run_shallow_depth_baseline import WHEELS, longest


def phase_windows(time, front_first, front_last, rear_first, rear_last, leading, trailing):
    def crossing(values, station):
        indices = np.flatnonzero(values >= station)
        return float(time[indices[0]]) if len(indices) else None
    events = dict(front_entry_s=crossing(front_first, leading-.263),
        front_exit_s=crossing(front_last, trailing+.263),
        rear_entry_s=crossing(rear_first, leading-.263),
        rear_exit_s=crossing(rear_last, trailing+.263),
        stop_target_s=crossing(rear_last, trailing+.263+.8))
    order = [events[k] for k in ['front_entry_s','front_exit_s','rear_entry_s','rear_exit_s']]
    observed = [x for x in order if x is not None]
    if observed != sorted(observed):
        raise ValueError('axle phase order changed; overlapping axle windows require another classifier')
    def interval(start, end):
        return np.zeros(len(time), dtype=bool) if start is None else ((time >= start) & (time < end if end is not None else True))
    phases = dict(front_axle=interval(order[0],order[1]), between_axles=interval(order[1],order[2]),
                  rear_axle=interval(order[2],order[3]), post_exit=interval(order[3],None))
    return phases, events


def episodes(time, selected):
    dt = float(np.median(np.diff(time)))
    starts = np.flatnonzero(selected & ~np.r_[False, selected[:-1]])
    ends = np.flatnonzero(selected & ~np.r_[selected[1:],False])
    return [dict(start_s=float(time[a]), end_s=float(time[b]), duration_s=float(time[b]-time[a]+dt)) for a,b in zip(starts,ends)]


def analyze(run):
    scene = json.loads((run/'model/scenario.json').read_text(encoding='utf-8'))
    manifest = json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
    contact = manifest.get('diagnostic') == 'CHASSIS_CONTACT_EXTENSION_V1'
    result_path = run / ('contact_diagnostic.json' if contact else 'baseline_diagnostic.json')
    raw = json.loads(result_path.read_text(encoding='utf-8'))
    if raw['native']['status'] != 'COMPLETED' or raw['native']['error_message']:
        raise ValueError('native failure: '+run.name)
    csv = run/('native_1ms.csv' if contact else 'native_5ms.csv')
    v = read_channels(csv); t = v['time_s']; dt = float(np.median(np.diff(t)))
    g = lambda k: v['exp_'+k]
    wide = run.parent.name == 'wide'
    front_first = np.maximum(g('X_L1'),g('X_R1')) if wide else g('X_R1')
    front_last = np.minimum(g('X_L1'),g('X_R1')) if wide else g('X_R1')
    rear_first = np.maximum(g('X_L2'),g('X_R2')) if wide else g('X_R2')
    rear_last = np.minimum(g('X_L2'),g('X_R2')) if wide else g('X_R2')
    phases, times = phase_windows(t,front_first,front_last,rear_first,rear_last,scene['start_station_m'],scene['start_station_m']+scene['length_m'])
    if times['front_entry_s'] is None:
        raise ValueError('not a crossing simulation: '+run.name)
    rear_exit = times['rear_exit_s']
    crossing = t >= times['front_entry_s']
    if rear_exit is not None:
        crossing &= t <= rear_exit+1.
    phases['crossing_plus_1s'] = crossing
    phases['crossing_plus_05s'] = (t>=times['front_entry_s']) & (t<=rear_exit+.5) if rear_exit is not None else crossing
    phases['full_after_entry'] = t >= times['front_entry_s']
    settled = (t>=4.) & (t<5.)
    y = g('Yo')-g('Yo')[settled].mean(); yaw = g('Yaw')-g('Yaw')[settled].mean()
    loss = np.stack([(g('Fz_'+w)<100.) & (g('CmpT_'+w+'i')<=.1) for w in WHEELS])
    support = 4-loss.sum(axis=0)
    az = g('Az_SM')*9.80665
    smooth = np.convolve(az,np.ones(int(round(.01/dt)))/int(round(.01/dt)),mode='same')
    phase_metrics = {}
    for phase, mask in phases.items():
        if not mask.any():
            phase_metrics[phase] = None; continue
        indices = np.flatnonzero(mask)
        def peak(values):
            i = indices[np.argmax(abs(values[mask]))]
            return dict(value=float(abs(values[i])),time_s=float(t[i]),signed_value=float(values[i]))
        metric = dict(start_s=float(t[indices[0]]),end_s=float(t[indices[-1]]),
            az_peak_m_s2=peak(az),az_10ms_peak_m_s2=peak(smooth),
            roll_peak_deg=peak(g('Roll_E')),pitch_peak_deg=peak(g('Pitch')),
            lateral_peak_m=peak(y),yaw_peak_deg=peak(yaw),yaw_rate_peak_deg_s=peak(g('AVz')),
            steer_sw_peak_deg=peak(v['imp_IMP_STEER_SW']),
            steer_saturation_s=float(np.sum(mask & (abs(v['imp_IMP_STEER_SW'])>=79.99))*dt),
            speed_min_kph=float(g('Vx')[mask].min()),speed_max_kph=float(g('Vx')[mask].max()),
            support_count_min=int(support[mask].min()),
            at_most_two_support_s=float(np.sum(mask&(support<=2))*dt),
            no_support_s=float(np.sum(mask&(support==0))*dt),
            drive_yaw_proxy_peak_nm=peak(.63*(g('Fx_R1')+g('Fx_R2')-g('Fx_L1')-g('Fx_L2'))), wheels={})
        for j,w in enumerate(WHEELS):
            loaded = (g('Fz_'+w)>=500.) & (abs(g('Vx'))>=.5)
            slip = loaded & (abs(g('Kappa_'+w+'i'))>.2)
            torque = v['imp_IMP_MYUSM_'+w]
            compression = g('Jnc_'+w)>=159.; rebound = g('Jnc_'+w)<=-149.
            wheel = dict(load_peak_n=peak(g('Fz_'+w)),
                jounce_max_mm=float(g('Jnc_'+w)[mask].max()),jounce_min_mm=float(g('Jnc_'+w)[mask].min()),
                loss_s=float(np.sum(mask&loss[j])*dt),longest_loss_s=longest(mask&loss[j],dt),
                compression_stop_s=float(np.sum(mask&compression)*dt),rebound_stop_s=float(np.sum(mask&rebound)*dt),
                loaded_large_slip_s=float(np.sum(mask&slip)*dt),
                loaded_large_slip_positive_torque_s=float(np.sum(mask&slip&(torque>1.))*dt),
                loaded_large_slip_negative_torque_s=float(np.sum(mask&slip&(torque<-1.))*dt),
                unloaded_positive_torque_s=float(np.sum(mask&loss[j]&(torque>1.))*dt),
                unloaded_speed_error_proxy_peak_m_s=float(np.max(abs(g('AVy_'+w)[mask&loss[j]]*2*np.pi/60*.263-g('Vx')[mask&loss[j]]/3.6))) if np.any(mask&loss[j]) else 0.)
            metric['wheels'][w] = wheel
        phase_metrics[phase] = metric
    flags = {'no_support':support==0,'at_most_two_support':support<=2,
        'steer_saturation':abs(v['imp_IMP_STEER_SW'])>=79.99,
        'large_pitch_over_20deg':abs(g('Pitch'))>20.,'large_roll_over_20deg':abs(g('Roll_E'))>20.}
    for j,w in enumerate(WHEELS):
        flags[w+'_loss'] = loss[j]
        flags[w+'_compression_stop'] = g('Jnc_'+w)>=159.
        flags[w+'_rebound_stop'] = g('Jnc_'+w)<=-149.
        flags[w+'_loaded_large_slip'] = (g('Fz_'+w)>=500.) & (abs(g('Vx'))>=.5) & (abs(g('Kappa_'+w+'i'))>.2)
    event_list = {k:episodes(t,mask&phases['full_after_entry']) for k,mask in flags.items()}
    contact_events = None
    if contact:
        count = raw['reference_point_count']
        inputs = np.stack([np.column_stack([v['imp_IMP_F%s_RP_%d'%(axis,i)] for axis in 'XYZ']) for i in range(1,count+1)],axis=1)
        # A fixed aggregate threshold avoids the legacy per-point >1 N rule,
        # whose effective aggregate detection threshold changes with grid size.
        magnitude_sum = np.linalg.norm(inputs,axis=2).sum(axis=1)
        selected = (magnitude_sum>10.) & phases['full_after_entry']
        contact_events = dict(definition='Sum of applied point-force magnitudes >10 N; sampled CSV, not native-step force duration',
            csv_dt_s=dt,first_s=float(t[np.flatnonzero(selected)[0]]) if selected.any() else None,
            duration_s=float(selected.sum()*dt),episodes=episodes(t,selected),
            legacy_native_per_point_events_not_used_for_grid_convergence=True)
    passive = all(np.max(abs(v['imp_IMP_FS_'+w]))==0. for w in WHEELS)
    equal = all(np.array_equal(v['imp_IMP_MYUSM_L1'],v['imp_IMP_MYUSM_'+w]) for w in WHEELS)
    if not passive or not equal:
        raise ValueError('baseline suspension or torque policy changed')
    return dict(run_path=str(run.relative_to(ROOT)),run_name=run.name,case=run.parent.name,
        depth_m=scene['depth_m'],target_speed_kph=manifest['diagnostic_options']['speed_kph'],
        depth_group='le_10cm' if scene['depth_m']<=.100001 else 'gt_10cm',
        source_type=('ASSUMED_CONTACT_DIAGNOSTIC' if raw['contact_enabled'] else 'REFERENCE_POINTS_NO_CONTACT_FORCE') if contact else 'ORIGINAL_NO_CHASSIS_FORCE_BASELINE',
        passive_additional_suspension_force=True,equal_wheel_torque=True,
        native=raw['native'],csv_sha256=hashlib.sha256(csv.read_bytes()).hexdigest(),
        base_vehicle_sha256=manifest['vehicle']['vehicle_sha256'],options=manifest['diagnostic_options'],
        time_events=times,phase_metrics=phase_metrics,events=event_list,
        crossing_plus_1s_censored=bool(rear_exit is None or t[-1]<rear_exit+1.-dt),
        crossing_plus_05s_censored=bool(rear_exit is None or t[-1]<rear_exit+.5-dt),
        observed_after_rear_exit_s=float(t[-1]-rear_exit) if rear_exit is not None else None,
        contact_result=raw if contact else None,aggregate_contact_events=contact_events,
        contact_config=manifest.get('contact_config'))


def summarize_convergence(rows, output):
    byname = {r['run_name']:r for r in rows}
    def metrics(r):
        raw=r['contact_result'];m=r['phase_metrics']['crossing_plus_1s']
        return dict(normal_force_peak_n=raw.get('normal_force_peak_n',0.),
            normal_impulse_ns=raw.get('normal_impulse_ns',0.),point_penetration_m=raw['maximum_point_penetration_m'],
            crossing_pitch_deg=m['pitch_peak_deg']['value'],crossing_roll_deg=m['roll_peak_deg']['value'],
            crossing_lateral_m=m['lateral_peak_m']['value'],az_10ms_m_s2=m['az_10ms_peak_m_s2']['value'],
            full_pitch_deg=raw['pitch_peak_deg'],full_roll_deg=raw['roll_peak_deg'],full_lateral_m=raw['lateral_peak_m'],
            rear_exit_s=r['time_events']['rear_exit_s'])
    pairs = [
        ('step_g45_500_to_250','chassis_contact_20261011_wide_d200_h150_grid45','contact_study_20261012_wide_d200_h150_g45_dt250'),
        ('step_g45_250_to_125','contact_study_20261012_wide_d200_h150_g45_dt250','contact_study_20261012_wide_d200_h150_g45_dt125'),
        ('step_g99_250_to_125','contact_study_20261012_wide_d200_h150_g99_dt250','contact_study_20261012_wide_d200_h150_g99_dt125'),
        ('grid_15_to_45_dt250','contact_study_20261012_wide_d200_h150_g15_dt250','contact_study_20261012_wide_d200_h150_g45_dt250'),
        ('grid_45_to_99_dt250','contact_study_20261012_wide_d200_h150_g45_dt250','contact_study_20261012_wide_d200_h150_g99_dt250'),
        ('grid_45_to_99_dt125','contact_study_20261012_wide_d200_h150_g45_dt125','contact_study_20261012_wide_d200_h150_g99_dt125'),
        ('stiffness_1_to_3_dt250','contact_study_20261012_wide_d200_h150_g15_dt250','contact_study_20261012_wide_d200_h150_g15_k3_dt250'),
        ('stiffness_3_to_10_dt250','contact_study_20261012_wide_d200_h150_g15_k3_dt250','contact_study_20261012_wide_d200_h150_g15_k10_dt250'),
        ('off_step_500_to_250','chassis_check_20261011_wide_d200_off','contact_study_20261012_wide_d200_h150_off_dt250'),
        ('off_step_250_to_125','contact_study_20261012_wide_d200_h150_off_dt250','contact_study_20261012_wide_d200_h150_off_dt125'),
        ('wide08_grid_and_step','chassis_contact_20261011_wide_d080_h100','contact_study_20261012_wide_d080_h100_g45_dt250'),
        ('right20_grid_and_step','chassis_contact_20261011_right_d200_h100','contact_study_20261012_right_d200_h100_g45_dt250'),
        ('wide16_grid_and_step','chassis_contact_20261011_wide_d160_h150','contact_study_20261012_wide_d160_h150_g45_dt250')]
    comparisons=[]
    for kind,a,b in pairs:
        x=metrics(byname[a]);y=metrics(byname[b]);changes={}
        for key in x:
            if x[key] is None or y[key] is None:
                changes[key]=None;continue
            changes[key]=dict(first=x[key],second=y[key],absolute_change=y[key]-x[key],
                relative_change=(y[key]-x[key])/abs(x[key]) if abs(x[key])>1e-9 else None)
        checked=['normal_force_peak_n','normal_impulse_ns','point_penetration_m','crossing_pitch_deg','crossing_roll_deg','az_10ms_m_s2']
        within = all(changes[k] is not None and abs(changes[k]['absolute_change'])<=max(.05*abs(x[k]),1. if k.endswith('_deg') else 0.) for k in checked)
        within &= changes['crossing_lateral_m'] is not None and abs(changes['crossing_lateral_m']['absolute_change'])<=max(.01,.1*abs(x['crossing_lateral_m']))
        physical_same = kind.startswith('step_') or kind.startswith('grid_')
        comparisons.append(dict(kind=kind,first_run=a,second_run=b,changes=changes,
            numerical_screen_pass=bool(within) if physical_same else None,
            note='Constitutive sensitivity, not numerical convergence' if kind.startswith('stiffness') else
                 ('Grid and step changed together: corroboration only' if 'grid_and_step' in kind else
                  'Solver and controller/CSV initial sample phases change with native step')))
    details=[]
    for r in rows:
        run=ROOT/r['run_path'];manifest=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
        assert hashlib.sha256((run/'model/run_all.par').read_bytes()).hexdigest()==manifest['extended_model_sha256']
        assert r['contact_result']['native_reference_frame_error_m']<1e-6
        assert r['native']['reached_configured_stop']
        if r['contact_result']['contact_enabled']:
            assert r['contact_result']['friction_work_j']<=0.
        details.append(dict(run_name=r['run_name'],case=r['case'],depth_m=r['depth_m'],options=r['options'],
            contact_config=r['contact_config'],source_type=r['source_type'],metrics=metrics(r),
            aggregate_contact_events=r['aggregate_contact_events'],csv_sha256=r['csv_sha256'],
            extended_model_sha256=manifest['extended_model_sha256'],base_vehicle_sha256=r['base_vehicle_sha256']))
    assert len({r['base_vehicle_sha256'] for r in rows})==1
    report=dict(scope='UNMEASURED_GEOMETRY_AND_UNCALIBRATED_COMPLIANT_CONTACT',runs=details,comparisons=comparisons,
        numerical_screen='5% changes in total force peak, impulse, point penetration, 10ms Az; pitch/roll max(5%,1deg); lateral max(10%,0.01m). Diagnostic screen, not certification.',
        limitations=['99-point spacing remains 0.2525m longitudinal and 0.125m lateral, larger than the 0.05m pit edge.',
                     'First control tick and CSV origin shift with step; CSV remains 1ms. Native accumulated force metrics retain each integration step.',
                     'Legacy per-point contact times/durations are not used for grid convergence; fixed 10N aggregate CSV events supplied instead.',
                     'Post-exit includes speed-PI stopping and steering recovery. Force and geometry assumptions do not represent measured real chassis.',
                     'First axle crossing phases do not classify a later return into the pit.'])
    (output/'convergence.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    lines=['# Contact sensitivity comparisons','','All full-duration runs: 30s. Parameters are uncalibrated.', '',
        '| Comparison | Fn peak change % | Normal impulse change % | Point penetration change % | Crossing pitch change deg | Numerical screen |',
        '|---|---:|---:|---:|---:|---|']
    for c in comparisons:
        d=c['changes']
        numbers=[('%.1f'%(d[k]['relative_change']*100)) if d[k]['relative_change'] is not None else 'n/a' for k in ['normal_force_peak_n','normal_impulse_ns','point_penetration_m']]
        lines.append('| %s | %s | %.2f | %s |'%(c['kind'],' | '.join(numbers),d['crossing_pitch_deg']['absolute_change'],c['numerical_screen_pass']))
    (output/'convergence_table.md').write_text('\n'.join(lines),encoding='utf-8')
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    groups=[('Step: 45 points / 1 MN/m',[pairs[0][1],pairs[0][2],pairs[1][2]],['0.5 ms','0.25 ms','0.125 ms']),
        ('Grid: 0.25 ms / 1 MN/m',[pairs[3][1],pairs[3][2],pairs[4][2]],['15 points','45 points','99 points']),
        ('Stiffness: 15 points / 0.25 ms',[pairs[6][1],pairs[6][2],pairs[7][2]],['1 MN/m','3 MN/m','10 MN/m']),
        ('No contact force: native step',[pairs[8][1],pairs[8][2],pairs[9][2]],['0.5 ms','0.25 ms','0.125 ms'])]
    for ax,(title,names,labels) in zip(axes.ravel(),groups):
        for name,label in zip(names,labels):
            r=byname[name];v=read_channels(ROOT/r['run_path']/'native_1ms.csv');t=v['time_s'];take=(t>=10.8)&(t<=16.)
            ax.plot(t[take],v['exp_Pitch'][take],label=label)
        ax.set(title=title,xlabel='Time (s)',ylabel='Pitch (deg)');ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(output/'contact_sensitivity.png',dpi=150);plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=ROOT/'evidence/passive_problem_study_20261012')
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError('choose a new output directory')
    sources = json.loads((ROOT/'evidence/speed_depth_baseline_20261011/summary.json').read_text(encoding='utf-8'))['results']
    original = [analyze(ROOT/r['run_path']) for r in sources]
    for r,source in zip(original,sources):
        assert r['csv_sha256']==source['csv_sha256']
    matrix = json.loads((ROOT/'configs/diagnostics/contact_study_matrix_20261012.json').read_text(encoding='utf-8'))
    for planned in matrix['runs']:
        run = ROOT/'runs/shallow_press'/planned['case']/planned['run_id']
        manifest = json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
        if manifest['run_status'] != 'DIAGNOSTIC_COMPLETED':
            raise ValueError('planned run incomplete: '+planned['run_id'])
        options = manifest['diagnostic_options']
        native = manifest['result']['native']
        if (options['speed_kph'] != matrix['speed_kph'] or options['duration_s'] != matrix['duration_s']
                or options['force_probe_n'] != 0. or native['configured_stop_s'] != matrix['duration_s']
                or not native['reached_configured_stop']):
            raise ValueError('planned speed/duration/probe mismatch: '+planned['run_id'])
        for key,value in planned.items():
            if key not in ['case','run_id'] and manifest['diagnostic_options'][key] != value:
                raise ValueError('planned option mismatch: '+planned['run_id']+' '+key)
    contacts = []
    for path in sorted((ROOT/'runs/shallow_press').glob('*/*/contact_diagnostic.json')):
        raw = json.loads(path.read_text(encoding='utf-8'))
        if raw['native']['final_time_s']>=29.9:
            contacts.append(analyze(path.parent))
    a.output.mkdir(parents=True)
    report = dict(scope='PASSIVE_SUSPENSION_EQUAL_TORQUE_SPEED_PI_AND_STRAIGHT_STEERING_FEEDBACK',
        thresholds=dict(contact_loss='Fz<100N and CmpT<=0.1mm',loaded_slip='Fz>=500N, speed>=0.5kph, |Kappa|>0.2',
                        compression_stop='Jnc>=159mm',rebound_stop='Jnc<=-149mm'),
        note='Thresholds label diagnostics, not safety acceptance. Drive yaw proxy is not net yaw moment. Post-exit includes stopping.',
        original_results=original,contact_results=contacts,planned_matrix=matrix,
        metric_code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (a.output/'phase_summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    summarize_convergence(contacts,a.output)
    lines = ['# Passive baseline phase diagnostics','','No active suspension or wheel-specific torque allocation; speed PI and steering feedback remain.', '',
        'Window: crossing through rear exit +1 s, or available prefix. Censored rows are not comparable at +1 s; use crossing_plus_05s in JSON for speed comparisons.', '',
        '| Case | km/h | Depth cm | Az 10ms m/s2 | Roll deg | Pitch deg | Wheel-loss sum s | Loaded slip sum s | Compression-stop sum s | Minimum support | +1s censored |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|']
    for r in original:
        m = r['phase_metrics']['crossing_plus_1s']; wheels = list(m['wheels'].values())
        lines.append('| %s | %.0f | %.0f | %.2f | %.2f | %.2f | %.3f | %.3f | %.3f | %d | %s |' % (
            r['case'],r['target_speed_kph'],r['depth_m']*100,m['az_10ms_peak_m_s2']['value'],m['roll_peak_deg']['value'],
            m['pitch_peak_deg']['value'],sum(w['loss_s'] for w in wheels),sum(w['loaded_large_slip_s'] for w in wheels),
            sum(w['compression_stop_s'] for w in wheels),m['support_count_min'],r['crossing_plus_1s_censored']))
    (a.output/'baseline_table.md').write_text('\n'.join(lines),encoding='utf-8')
    lines = ['# Retrieval index','','| Run | Source | Front entry s | Front exit s | Rear entry s | Rear exit s | Stop target s |', '|---|---|---:|---:|---:|---:|---:|']
    for r in original+contacts:
        e = r['time_events']
        values = [('%.4f'%e[k]) if e[k] is not None else 'unobserved' for k in ['front_entry_s','front_exit_s','rear_entry_s','rear_exit_s','stop_target_s']]
        lines.append('| `%s` | %s | %s |'%(r['run_name'],r['source_type'],' | '.join(values)))
    (a.output/'case_index.md').write_text('\n'.join(lines),encoding='utf-8')
    selected = [r for r in original if r['target_speed_kph']==10. and ((r['case']=='right_track' and r['depth_m']==.2) or (r['case']=='wide' and r['depth_m'] in [.08,.10,.16,.20]))]
    fig,axes = plt.subplots(len(selected),3,figsize=(13,2.6*len(selected)))
    for row,r in enumerate(selected):
        v = read_channels(ROOT/r['run_path']/'native_5ms.csv'); t=v['time_s']; e=r['time_events']
        end = min(t[-1],e['rear_exit_s']+2.)
        take = (t>=e['front_entry_s']-.2)&(t<=end)
        axes[row,0].plot(t[take],v['exp_Az_SM'][take]*9.80665,label='Az sprung (raw)')
        axes[row,1].plot(t[take],v['exp_Pitch'][take],label='Pitch')
        axes[row,1].plot(t[take],v['exp_Roll_E'][take],label='Roll')
        for w in WHEELS:
            axes[row,2].plot(t[take],v['exp_Fz_'+w][take]/1000,label=w)
        for ax in axes[row]:
            for key in ['front_entry_s','front_exit_s','rear_entry_s','rear_exit_s']:
                ax.axvline(e[key],color='k',alpha=.3,linestyle=':')
            ax.grid(alpha=.2); ax.legend(fontsize=7); ax.set_xlabel('Time (s)')
        axes[row,0].set_ylabel('%s %dcm\nAz (m/s2)'%(r['case'],r['depth_m']*100))
        axes[row,1].set_ylabel('Attitude (deg)'); axes[row,2].set_ylabel('Wheel load (kN)')
    fig.tight_layout();fig.savefig(a.output/'representative_phases.png',dpi=140);plt.close(fig)
    print(json.dumps(dict(original_runs=len(original),contact_runs=len(contacts),output=str(a.output))))


if __name__=='__main__':
    main()
