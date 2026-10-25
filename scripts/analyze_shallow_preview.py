"""Verify immutable run evidence, summarize a frozen matrix and plot exit progress."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def source_digest_map(files):
    """Canonical relative source names; reject aliases instead of hiding drift."""
    result={}
    for name,digest in files.items():
        key=name.replace('\\','/').casefold()
        if ':' in key or any(p in ('','.','..') for p in key.split('/')):
            raise ValueError('non-canonical relative source path')
        if key in result:raise ValueError('duplicate source path alias')
        result[key]=digest
    return result


def verified_run(run):
    manifest=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
    result=json.loads((run/'result.json').read_text(encoding='utf-8'))
    checks={'result':sha(run/'result.json')==manifest['result_sha256'],
        'native':sha(run/'native_1ms.csv')==result['native_csv_sha256'],
        'model':sha(run/'model/run_all.par')==result['model_sha256']==manifest['final_model_sha256'],
        'provenance':sha(run/'input_provenance.json')==result['input_provenance_sha256']==manifest['input_provenance_sha256'],
        'plant':result['plant']==manifest['plant'],
        'complete':manifest['run_status']=='COMPLETED' and result['native']['reached_configured_stop']}
    for prefix,files in (('',manifest['generated_assets']),('source/',manifest['source_sha256'])):
        for name,expected in files.items():checks[prefix+name]=sha(run/(prefix+name))==expected
    failed=[k for k,v in checks.items() if not v]
    if failed:raise ValueError('evidence mismatch '+','.join(failed)+' in '+str(run))
    return result,manifest


def read_columns(run,columns):
    import numpy as np
    path=run/'native_1ms.csv'
    with path.open(encoding='utf-8') as stream:names=stream.readline().strip().split(',')
    data=np.genfromtxt(path,delimiter=',',skip_header=1,usecols=[names.index(n) for n in columns])
    return dict(zip(columns,data.T))


def verified_historical_run(run):
    """Earlier development runs predate portable provenance; never rewrite them."""
    manifest=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'))
    result=json.loads((run/'result.json').read_text(encoding='utf-8'))
    if (sha(run/'result.json')!=manifest['result_sha256'] or
            sha(run/'native_1ms.csv')!=result['native_csv_sha256'] or
            sha(run/'model/run_all.par')!=result['model_sha256'] or
            not result['native']['reached_configured_stop']):
        raise ValueError('historical evidence mismatch')
    for name,expected in manifest['source_sha256'].items():
        if sha(run/'source'/name)!=expected:raise ValueError('historical source mismatch')
    return result


def plot_comparison(before,after,output):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from ddevsim.strategies.shallow_press.assessment import exit_progress_metrics
    cols=['time_s','exp_Vx','exp_Az_SM','imp_IMP_FS_R1']
    cols += ['exp_'+axis+'_'+w for w in ('L1','R1','L2','R2') for axis in ('X','Y')]
    cols += ['imp_IMP_MYUSM_'+w for w in ('L1','R1','L2','R2')]
    figure,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    comparison={}
    for run,label,color in ((before,'Before','#b44738'),(after,'Optimized','#14608c')):
        result=verified_historical_run(run) if run==before else verified_run(run)[0]
        d=read_columns(run,cols);scene=result['scene']
        end=scene['start_station_m']+scene['length_m'];t=d['time_s']
        begin=t[np.flatnonzero(d['exp_X_R1']>end-.263)[0]]
        sample=(t>=begin-1)&(t<=begin+5);relative=t[sample]-begin
        axes[0].plot(relative,d['exp_Vx'][sample],label=label,color=color)
        axes[1].plot(relative,sum(d['imp_IMP_MYUSM_'+w] for w in ('L1','R1','L2','R2'))[sample],color=color)
        axes[2].plot(relative,d['imp_IMP_FS_R1'][sample],color=color)
        progress=exit_progress_metrics(d,scene,result['config']['crossing_kph'])
        comparison[label]=dict(run=str(run.relative_to(ROOT)),exit_progress=progress,
            metrics=result['metrics'],result_sha256=sha(run/'result.json'),
            provenance_scope='HISTORICAL_WITHOUT_PORTABLE_PROVENANCE' if run==before else 'FROZEN_INPUT_PROVENANCE')
    axes[0].axhline(.2,color='gray',ls=':',label='0.2 km/h')
    axes[0].set_ylabel('Vehicle speed (km/h)');axes[0].legend(loc='upper right')
    axes[1].set_ylabel('Sum motor torque (Nm)');axes[2].set_ylabel('FR suspension force (N)')
    axes[2].set_xlabel('Time from FR entering exit zone (s)')
    for axis in axes:axis.grid(alpha=.25);axis.axvline(0,color='gray',lw=.7)
    figure.suptitle('Right-track pit: depth 4 cm, target 1 km/h\nMF52 comparison plant; body feedback delay 40 ms; not full safety acceptance')
    figure.tight_layout();figure.savefig(output/'exit_optimization.png',dpi=160);plt.close(figure)
    (output/'exit_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n',encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence',required=True,type=Path)
    parser.add_argument('--before-run',type=Path)
    parser.add_argument('--after-run',type=Path)
    args=parser.parse_args();evidence=args.evidence
    plan_path=evidence/'frozen_plan.json';plan=json.loads(plan_path.read_text(encoding='utf-8'))
    rows=[];reference=None
    for case in plan['cases']:
        run=ROOT/'runs/shallow_press'/case['case']/case['run_id'];result,manifest=verified_run(run)
        marker=json.loads((run/'matrix_identity.json').read_text(encoding='utf-8'))
        if marker!=dict(plan_sha256=sha(plan_path),args=case['args']):raise ValueError('matrix identity mismatch')
        actual=source_digest_map(manifest['source_sha256'])
        if reference is None:reference=actual
        if actual!=reference:raise ValueError('matrix source drift')
        if any(actual.get(name)!=expected for name,expected in source_digest_map(plan['source_sha256']).items() if Path(name).name!='__init__.py'):
            raise ValueError('run does not match frozen sources')
        rows.append(dict(**{k:case[k] for k in ('case','run_id','kind','depth_m','crossing_kph')},
            status=result['status'],failed_criteria=[k for k,v in result['criteria'].items() if not v],
            metrics=result['metrics'],exit_progress=result['exit_progress'],
            chassis_diagnostic=result['chassis_diagnostic'],plant=result['plant'],
            capability=result.get('capability'),diagnostic_motion_pass=result.get('diagnostic_motion_pass'),
            diagnostic_failed_criteria=[k for k,v in result.get('diagnostic_criteria',{}).items() if not v],
            result_sha256=sha(run/'result.json'),input_provenance_sha256=manifest['input_provenance_sha256'],
            native_history_sha256={str(p.relative_to(run)):sha(p) for p in sorted((run/'model/output').glob('*.vs*'))}))
    output=dict(scope=plan['scope'],plan_sha256=sha(plan_path),source_sha256=reference,
        config_sha256=plan['config_sha256'],count=len(rows),passing=sum(r['status']=='SIMULATION_PASS' for r in rows),
        full_acceptance=False,rows=rows)
    (evidence/'verified_analysis.json').write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
    lines=['# 固定参数验证工况索引','',
        '所有视频为原生历史，未导出视频文件。MF52采纳身份以各运行plant元数据为准；SIMULATION_PASS仅指参数化研究筛查，能力扫描不计入基线通过。', '',
        '|工况名称|类别|坑深cm|目标km/h|筛查|未通过项目|', '|---|---|---:|---:|---|---|']
    for row in rows:
        lines.append('|%s|%s/%s|%g|%g|%s|%s|'%(
            row['run_id'],row['case'],row['kind'],row['depth_m']*100,row['crossing_kph'],
            row['status'],', '.join(row['failed_criteria']) or '—'))
    lines+=['','播放：`python scripts/open_run_visualizer.py runs/shallow_press/<case>/<run_id>`。',
            '详细速度、出坑停顿、姿态、轮速误差、轮载、净空假设及哈希见`verified_analysis.json`。']
    (evidence/'case_index.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    if any(row['diagnostic_motion_pass'] is not None for row in rows):
        lines=['# 独立转矩能力扫描','',
            '原能力基线保持1000 Nm/s。以下只放宽该次诊断的转矩变化率，转矩幅值仍为±500 Nm；全部simulation_pass=false。', '',
            '|工况名称|变化率Nm/s|实际均速km/h|轮速误差m/s|最大轮载/静载|诊断联合筛查|诊断未通过项目|',
            '|---|---:|---:|---:|---:|---|---|']
        for row in rows:
            m=row['metrics']
            lines.append('|%s|%g|%.3f|%.3f|%.3f|%s|%s|' % (row['run_id'],
                row['capability']['actual_torque_rate_nm_s'],m['crossing_mean_kph'],
                m['loaded_slip_proxy_peak_m_s'],max(m['wheel_load_peak_ratio_to_static']),
                row['diagnostic_motion_pass'],', '.join(row['diagnostic_failed_criteria']) or '—'))
        (evidence/'case_index.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    if bool(args.before_run)!=bool(args.after_run):parser.error('comparison requires both before/after runs')
    if args.before_run:plot_comparison(args.before_run.resolve(),args.after_run.resolve(),evidence)
    print(json.dumps(dict(count=len(rows),passing=output['passing'],full_acceptance=False)))
    return 0


if __name__=='__main__':raise SystemExit(main())
