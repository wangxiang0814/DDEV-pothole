"""Analyse native speed/depth diagnostics including one second after rear exit.

No expert acceptance or chassis-contact certification is inferred here.
"""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from run_shallow_depth_baseline import ROOT, WHEELS, longest


def read_channels(path):
    with path.open(encoding='utf-8') as stream:
        names = stream.readline().strip().split(',')
        values = np.loadtxt(stream, delimiter=',', ndmin=2)
    return {k: values[:,i] for i,k in enumerate(names)}


def extended_window(time, front, rear, leading, trailing, radius=.263):
    entries = np.flatnonzero(front >= leading-radius)
    exits = np.flatnonzero(rear > trailing+radius)
    if not len(entries):
        raise ValueError('no front-entry observation')
    start = float(time[entries[0]])
    if not len(exits):
        return time >= start, start, None
    end = float(time[exits[0]])
    return (time >= start) & (time <= end+1.), start, end


def analyse(run):
    manifest = json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
    result = json.loads((run/'baseline_diagnostic.json').read_text(encoding='utf-8'))
    if manifest['run_status'] != 'DIAGNOSTIC_COMPLETED' or result['native']['status'] != 'COMPLETED':
        raise ValueError('invalid native diagnostic: '+str(run))
    scene = json.loads((run/'model/scenario.json').read_text(encoding='utf-8'))
    v = read_channels(run/'native_5ms.csv')
    get = lambda k: v['exp_'+k]
    t = v['time_s']; dt = float(np.median(np.diff(t)))
    wide = result['case'] == 'wide'
    front = np.maximum(get('X_L1'), get('X_R1')) if wide else get('X_R1')
    rear = np.minimum(get('X_L2'), get('X_R2')) if wide else get('X_R2')
    leading = scene['start_station_m']; trailing = leading+scene['length_m']
    mask, entry, exit_time = extended_window(t,front,rear,leading,trailing)
    pre = (front >= leading-.8) & (front < leading-.3)
    settled = (t>=4.) & (t<5.)
    result.update(run_id=run.name,run_path=str(run.relative_to(ROOT)),
                  options=manifest['diagnostic_options'],vehicle_sha256=scene['vehicle']['vehicle_sha256'],
                  csv_sha256=hashlib.sha256((run/'native_5ms.csv').read_bytes()).hexdigest(),
                  dt_s=dt,analysis_start_s=entry,rear_exit_s=exit_time,
                  analysis_censored=bool(exit_time is None or t[-1]<exit_time+1.),
                  crossing_outcome='PROTECTIVE_ABORT' if result['abort_reason'] else ('REAR_CLEARED' if exit_time is not None else 'TIMEOUT_NOT_CLEARED'),
                  analysis_end_s=float(t[mask][-1]),
                  entry_speed_kph=float(get('Vx')[np.flatnonzero(front>=leading)[0]]),
                  pre_speed_min_kph=float(get('Vx')[pre].min()),
                  pre_speed_max_kph=float(get('Vx')[pre].max()))
    def peak(key, data):
        idx = np.flatnonzero(mask)[np.argmax(np.abs(data[mask]))]
        result[key] = float(abs(data[idx])); result[key+'_time_s']=float(t[idx])
    peak('window_az_peak_m_s2',get('Az_SM')*9.80665)
    n=max(1,int(round(.01/dt)))
    peak('az_10ms_mean_peak_m_s2',np.convolve(get('Az_SM')*9.80665,np.ones(n)/n,mode='same'))
    peak('window_roll_peak_deg',get('Roll_E'))
    peak('window_pitch_peak_deg',get('Pitch'))
    peak('yaw_rate_peak_deg_s',get('AVz'))
    peak('window_lateral_peak_m',get('Yo')-get('Yo')[settled].mean())
    peak('window_yaw_peak_deg',get('Yaw')-get('Yaw')[settled].mean())
    peak('steering_sw_peak_deg',v['imp_IMP_STEER_SW'])
    peak('drive_yaw_proxy_peak_nm',.63*(get('Fx_R1')+get('Fx_R2')-get('Fx_L1')-get('Fx_L2')))
    # Fx is wheel-longitudinal, so the preceding is ONLY a straight-line
    # differential-traction proxy; not the net yaw moment (Fy/Mz omitted).
    result['window_speed_min_kph']=float(get('Vx')[mask].min())
    result['window_speed_max_kph']=float(get('Vx')[mask].max())
    losses=[]; jounce=[]; loads=[]
    for w in WHEELS:
        loss=(get('Fz_'+w)<100.) & (get('CmpT_'+w+'i')<=.1)
        loaded=(get('Fz_'+w)>=500.) & (abs(get('Vx'))>=.5)
        slip=abs(get('Kappa_'+w+'i'))
        indices=np.flatnonzero(mask)
        imax=indices[np.argmax(get('Jnc_'+w)[mask])]
        result['wheels'][w].update(
            window_contact_loss_s=float(np.sum(mask&loss)*dt),
            window_longest_contact_loss_s=longest(mask&loss,dt),
            window_load_max_n=float(get('Fz_'+w)[mask].max()),
            window_jounce_max_mm=float(get('Jnc_'+w)[mask].max()),
            jounce_peak_time_s=float(t[imax]),
            window_jounce_min_mm=float(get('Jnc_'+w)[mask].min()),
            compression_stop_peak_mm=float(get('CmpJSt'+w)[mask].max()),
            compression_stop_s=float(np.sum(mask&(get('Jnc_'+w)>=159.))*dt),
            rebound_stop_peak_mm=float(get('CmpRSt'+w)[mask].max()),
            rebound_stop_s=float(np.sum(mask&(get('Jnc_'+w)<=-149.))*dt),
            loaded_slip_over_02_window_s=float(np.sum(mask&loaded&(slip>.2))*dt),
            loaded_slip_peak_window=float(slip[mask&loaded].max()) if np.any(mask&loaded) else None,
            unloaded_spin_error_peak_m_s=float(np.max(abs(get('AVy_'+w)[mask&loss]*2*np.pi/60*.263-get('Vx')[mask&loss]/3.6))) if np.any(mask&loss) else 0.)
        losses.append(loss);jounce.append(get('Jnc_'+w));loads.append(get('Fz_'+w))
    support=4-np.stack(losses).sum(axis=0)
    result['support_count_min']=int(support[mask].min())
    result['support_at_most_two_s']=float(np.sum(mask&(support<=2))*dt)
    result['wheel_contact_loss_sum_window_s']=sum(w['window_contact_loss_s'] for w in result['wheels'].values())
    result['window_jounce_peak_mm']=float(np.stack(jounce)[:,mask].max())
    result['window_load_peak_n']=float(np.stack(loads)[:,mask].max())
    # Both speeds fully cover 0.5 s of rear-exit aftermath; use this common
    # horizon for speed comparisons. Protected aborts retain available data.
    common=mask if exit_time is None else (t>=entry)&(t<=exit_time+.5)
    result['comparison_censored']=bool(exit_time is None or t[-1]<exit_time+.5)
    for key,data in [('window_az_peak_m_s2',get('Az_SM')*9.80665),
                     ('window_roll_peak_deg',get('Roll_E')),('window_pitch_peak_deg',get('Pitch')),
                     ('yaw_rate_peak_deg_s',get('AVz'))]:
        result['common05_'+key]=float(np.max(abs(data[common])))
    result['common05_window_jounce_peak_mm']=float(np.stack(jounce)[:,common].max())
    result['common05_wheel_contact_loss_sum_window_s']=float(np.sum(np.stack(losses)[:,common])*dt)
    result['depth_group']='le_10cm' if result['depth_m']<=.100001 else 'gt_10cm'
    full = t >= entry
    result['simulation_end_s'] = float(t[-1])
    result['terminal_speed_kph'] = float(get('Vx')[-1])
    result['terminal_lateral_m'] = float(get('Yo')[-1]-get('Yo')[settled].mean())
    for key,data in [('roll_deg',get('Roll_E')),('pitch_deg',get('Pitch')),
                     ('lateral_m',get('Yo')-get('Yo')[settled].mean()),
                     ('yaw_deg',get('Yaw')-get('Yaw')[settled].mean())]:
        idx = np.flatnonzero(full)[np.argmax(abs(data[full]))]
        result['full_'+key+'_peak'] = float(abs(data[idx]))
        result['full_'+key+'_peak_time_s'] = float(t[idx])
    return result


def main():
    output=ROOT/'evidence/speed_depth_baseline_20261011';output.mkdir(exist_ok=True)
    results=[analyse(p.parent) for p in sorted((ROOT/'runs/shallow_press').glob('*/speed_study_20261011_*/baseline_diagnostic.json'))]
    if len(results)!=18:
        raise ValueError('expected 12 new-speed + 6 matched-low-speed diagnostics')
    protected_results = [r for r in results if r['abort_reason']]
    continuations = [analyse(p.parent) for p in sorted((ROOT/'runs/shallow_press').glob('*/speed_study_full_20261011_*/baseline_diagnostic.json'))]
    extended = [analyse(p.parent) for p in sorted((ROOT/'runs/shallow_press').glob('*/speed_study_full30_20261011_*/baseline_diagnostic.json'))]
    for continuation in continuations + extended:
        key = (continuation['case'], continuation['depth_m'], continuation['target_speed_kph'])
        matches = [i for i,r in enumerate(results) if (r['case'],r['depth_m'],r['target_speed_kph']) == key]
        if len(matches) != 1:
            raise ValueError('continuation must replace exactly one original condition')
        results[matches[0]] = continuation
    if len({r['vehicle_sha256'] for r in results})!=1:
        raise ValueError('plant identity changed')
    high=[r for r in results if r['target_speed_kph']==10.]
    if not all(9.5<=r['pre_speed_min_kph']<=r['pre_speed_max_kph']<=10.5 for r in high):
        raise ValueError('pre-entry speed outside declared 10 +/- .5 km/h band')
    report={'scope':'native diagnostic, passive suspension; no expert PASS',
            'window':'first front tyre envelope entry to last rear envelope exit + 1 s',
            'comparison_window':'same envelope crossing + 0.5 s; censored failures marked with x',
            'stop_detection':'Jnc>=159 mm compression table onset; Jnc<=-149 mm rebound onset; Cmp*St are coordinates, NOT zero-based compression',
            'yaw_moment':'drive_yaw_proxy is half-track * wheel-longitudinal right-minus-left force; NOT net yaw moment',
            'chassis_contact':'NOT_OBSERVABLE','results':results,
            'original_protected_runs':protected_results,
            'continuation_count':len(continuations)+len(extended),
            'executed_run_count':18+len(continuations)+len(extended),
            'intermediate_15s_runs':continuations,
            'metric_code_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (output/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    index = []
    lines = ['# 工况名称与后续重放索引', '',
             '本轮不播放或导出视频。以下是18个工况的最终分析来源；原截断及15秒重跑保留供溯源。', '',
             '所有工况：纵长0.8 m、坑前12 m、μ=0.7、被动悬架、直行转向反馈。单侧宽0.9 m，宽坑宽1.8 m。', '',
             '| 工况名称 | 运行名称 | 前轮入坑/后轮出坑 s | 冲击/俯仰峰值时刻 s |',
             '|---|---|---|---|']
    for r in results:
        title='%s_v%02dkph_d%02dcm_W%dmm_L800mm'%('RIGHT' if r['case']=='right_track' else 'WIDE',r['target_speed_kph'],round(r['depth_m']*100),round(r['width_m']*1000))
        if r['options'].get('full_duration'):
            title += '_FULL30s'
        history = list((ROOT/r['run_path']/'model/output').glob('*.vsb'))
        if len(history) != 1:
            raise ValueError('exactly one native animation history required')
        record={'case_name':title,'run_id':r['run_id'],'case':r['case'],'depth_m':r['depth_m'],
                'speed_kph':r['target_speed_kph'],'source_run':r['run_path'],
                'absolute_run':str(ROOT/r['run_path']),
                'native_vsb_sha256':hashlib.sha256(history[0].read_bytes()).hexdigest(),
                'simulation_end_s':r['simulation_end_s'],'crossing_outcome':r['crossing_outcome'],
                'critical_times_s':{'front_entry':r['analysis_start_s'],'rear_exit':r['rear_exit_s'],
                                    'impact_peak':r['window_az_peak_m_s2_time_s'],
                                    'pitch_peak':r['full_pitch_deg_peak_time_s'],
                                    'roll_peak':r['full_roll_deg_peak_time_s']},
                'replay_command':'F:/anaconda3/python.exe scripts/open_run_visualizer.py '+r['run_path'].replace('\\','/')}
        index.append(record)
        exit_label = '未出坑' if r['rear_exit_s'] is None else '%.3f'%r['rear_exit_s']
        lines.append('| %s | `%s` | %.3f / %s | %.3f / %.3f |'%(title,r['run_id'],r['analysis_start_s'],exit_label,r['window_az_peak_m_s2_time_s'],r['full_pitch_deg_peak_time_s']))
    lines += ['', '运行目录为 `runs/shallow_press/<right_track或wide>/<运行名称>/`。',
              '后续打开方式：`F:/anaconda3/python.exe scripts/open_run_visualizer.py runs/shallow_press/<坑型>/<运行名称>`，一次打开一个工况。',
              'JSON保存完整路径、原生历史哈希和关键时刻。动画采样25 ms，峰值判断以1 ms CSV为准。', '']
    (output/'case_index.md').write_text('\n'.join(lines),encoding='utf-8')
    (output/'video_index.json').write_text(json.dumps({'video_export':'NOT_REQUESTED',
            'playback':'DEFERRED_BY_USER','runs':index},ensure_ascii=False,indent=2),encoding='utf-8')
    keys=['case','run_id','depth_m','depth_group','target_speed_kph','entry_speed_kph','pre_speed_min_kph','pre_speed_max_kph','rear_cleared','crossing_outcome','abort_reason','analysis_censored','window_az_peak_m_s2','az_10ms_mean_peak_m_s2','window_roll_peak_deg','window_pitch_peak_deg','window_jounce_peak_mm','window_load_peak_n','wheel_contact_loss_sum_window_s','support_count_min','support_at_most_two_s','window_yaw_peak_deg','yaw_rate_peak_deg_s','window_lateral_peak_m','drive_yaw_proxy_peak_nm','steering_sw_peak_deg','analysis_start_s','rear_exit_s','analysis_end_s']
    with (output/'summary.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,keys);writer.writeheader();writer.writerows({k:r[k] for k in keys} for r in results)
    fig,axes=plt.subplots(2,3,figsize=(13,7),constrained_layout=True)
    metrics=[('window_az_peak_m_s2','Peak sprung acceleration (m/s²)'),('window_jounce_peak_mm','Maximum jounce (mm)'),('wheel_contact_loss_sum_window_s','Summed contact loss (wheel·s)'),('window_roll_peak_deg','Absolute roll (deg)'),('window_pitch_peak_deg','Absolute pitch (deg)'),('yaw_rate_peak_deg_s','Yaw rate (deg/s)')]
    for ax,(key,label) in zip(axes.flat,metrics):
        for case,color in [('right_track','#2563eb'),('wide','#dc2626')]:
            for speed,style in [(10.,'o-'),(3.,'s--')]:
                data=sorted([r for r in results if r['case']==case and r['target_speed_kph']==speed],key=lambda r:r['depth_m'])
                complete=[r for r in data if not r['comparison_censored']]
                failed=[r for r in data if r['comparison_censored']]
                ax.plot([r['depth_m']*100 for r in complete],[r['common05_'+key] for r in complete],style,color=color,label='%s %g km/h'%(case,speed))
                if failed:
                    ax.plot([r['depth_m']*100 for r in failed],[r['common05_'+key] for r in failed],'x',color=color,markersize=9,label='protected abort (partial)')
        ax.axvline(10,color='#64748b',linestyle=':');ax.set(xlabel='Pit depth (cm)',ylabel=label);ax.grid(alpha=.25)
    axes[0,0].legend(fontsize=8)
    fig.suptitle('Native passive baseline: matched 12 m approach, straight steering feedback\n0.8 m pit; 0.05 m edges; common +0.5 s rear-exit horizon; x = censored abort; 10 cm = research split')
    fig.savefig(output/'speed_depth_sweep.png',dpi=160);plt.close(fig)
    for r in results:
        print('%s %g cm %g km/h: entry %.3f Az %.2f roll %.2f pitch %.2f Jnc %.1f loss %.3f yawrate %.2f y %.4f'%(r['case'],r['depth_m']*100,r['target_speed_kph'],r['entry_speed_kph'],r['window_az_peak_m_s2'],r['window_roll_peak_deg'],r['window_pitch_peak_deg'],r['window_jounce_peak_mm'],r['wheel_contact_loss_sum_window_s'],r['yaw_rate_peak_deg_s'],r['window_lateral_peak_m']))


if __name__=='__main__':
    main()
