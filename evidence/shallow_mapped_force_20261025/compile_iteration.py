"""Compile this immutable research iteration from verified native run histories.

Requires the retained local runs; source bundles are provenance, not a standalone
TruckSim installation. Never changes a frozen plan, source, model or native run.
"""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_shallow_preview import verified_run,source_digest_map,sha


def read(path):return json.loads(path.read_text(encoding='utf-8'))


def main():
    base=Path(__file__).resolve().parent
    archives=base/'source_archives';archives.mkdir(exist_ok=True)
    snapshots=base/'run_snapshots';snapshots.mkdir(exist_ok=True)
    baseline=ROOT/'runs/shallow_press/wide/coverage_v2yaw_wide_d10_v10_20261025_001'
    result,manifest=verified_run(baseline)
    rows=[dict(label='retained_v2',run=str(baseline.relative_to(ROOT)),result=result,
               result_sha256=sha(baseline/'result.json'),native_sha256=sha(baseline/'native_1ms.csv'))]
    for plan_path in sorted(base.glob('*/frozen_plan.json')):
        plan=read(plan_path);config_path=ROOT/plan['config_path']
        if sha(config_path)!=plan['config_sha256']:raise ValueError('config drift')
        expected=source_digest_map(plan['source_sha256'])
        for case in plan['cases']:
            run=ROOT/'runs/shallow_press'/case['case']/case['run_id']
            r,m=verified_run(run)
            if read(run/'matrix_identity.json')!=dict(plan_sha256=sha(plan_path),args=case['args']):raise ValueError('plan binding')
            actual=source_digest_map(m['source_sha256'])
            if actual!=expected:raise ValueError('source binding')
            if r['config']!=m['effective_config']:raise ValueError('config mismatch')
            cfg=read(config_path)
            for option,key in (('--speed-kph','cruise_kph'),('--crossing-kph','crossing_kph'),('--mode','mode')):
                if option in case['args']:
                    value=case['args'][case['args'].index(option)+1]
                    cfg[key]=value if key=='mode' else float(value)
            if any(r['config'].get(k)!=v for k,v in cfg.items()):raise ValueError('explicit config mismatch')
            if r['scene']['depth_m']!=case['depth_m'] or r['config']['crossing_kph']!=case['crossing_kph']:raise ValueError('case mismatch')
            if r['simulation_pass']!=all(r['criteria'].values()) or r['acceptance_pass']:raise ValueError('invalid screen status')
            history={str(p.relative_to(run)):sha(p) for p in sorted((run/'model/output').glob('*.vs*'))}
            if not history:raise ValueError('native playback history missing')
            digest=hashlib.sha256(json.dumps(actual,sort_keys=True).encode('utf-8')).hexdigest()
            archive=archives/(digest[:16]+'.zip')
            if not archive.exists():
                with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as z:
                    for name in sorted(actual):z.write(run/'source'/name,name)
            with zipfile.ZipFile(archive) as z:
                for name,h in actual.items():
                    if hashlib.sha256(z.read(name)).hexdigest()!=h:raise ValueError('source archive mismatch')
            snapshot=snapshots/case['run_id'];snapshot.mkdir(exist_ok=True)
            for name in ('result.json','input_provenance.json','run_manifest.json','control_config.json'):
                target=snapshot/name;content=(run/name).read_bytes()
                if target.exists() and target.read_bytes()!=content:raise ValueError('snapshot refuses overwrite')
                if not target.exists():target.write_bytes(content)
            rows.append(dict(label=plan_path.parent.name,case=case['case'],run=str(run.relative_to(ROOT)),
                plan_path=str(plan_path.relative_to(ROOT)),plan_sha256=sha(plan_path),
                result=r,result_sha256=sha(run/'result.json'),native_sha256=sha(run/'native_1ms.csv'),
                native_history_sha256=history,source_archive=str(archive.relative_to(ROOT)),source_archive_sha256=sha(archive)))
    output=dict(scope='DEEP_FIRST_TARGETED_CONTROL_DIAGNOSTICS_NOT_ADOPTED_PARAMETERS',
                native_completed_count=len(rows)-1,passing_new_count=sum(x['result']['simulation_pass'] for x in rows[1:]),
                baseline_excluded_from_new_count=True,full_acceptance=False,compiler_sha256=sha(Path(__file__)),rows=rows)
    (base/'iteration_summary.json').write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
    lines=['# 本轮工况索引','',
        '均为MF52研究模型、原软件能力下的原生运行。只统计本轮新运行；retained_v2为历史对照。实际均速是闭环结果，不是严格同轨迹的单独悬架辨识。', '',
        '|组别|工况名|目标km/h|实际均速km/h|RMS m/s²|冲击10ms m/s²|轮载/静载|轮速差m/s|最长失载ms|未通过|',
        '|---|---|---:|---:|---:|---:|---:|---:|---:|---|']
    for row in rows:
        r=row['result'];m=r['metrics']
        lines.append('|%s|%s|%g|%.3f|%.3f|%.3f|%.3f|%.3f|%.0f|%s|'%(row['label'],Path(row['run']).name,
            r['config']['crossing_kph'],m['crossing_mean_kph'],m['az_rms_m_s2'],m['az_10ms_peak_m_s2'],
            max(m['wheel_load_peak_ratio_to_static']),m['loaded_slip_proxy_peak_m_s'],
            max(m['longest_contact_loss_s'])*1000,', '.join(k for k,v in r['criteria'].items() if not v) or '—'))
    lines+=['','播放对应run目录：`python scripts/open_run_visualizer.py <run目录>`。原生视频历史保留，没有导出视频。',
        '精确配置、失败项目及来源/原生历史哈希见 iteration_summary.json；源码ZIP不包括TruckSim安装、授权或完整仿真原始数据。']
    (base/'case_index.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    subset=[row for row in rows if row['result']['scene']['depth_m']==.1 and row['result']['config']['crossing_kph']==1 and row.get('case','wide')=='wide']
    fig,axes=plt.subplots(2,2,figsize=(14,8))
    labels=[row['label'] for row in subset]
    for ax,(label,fn,limit) in zip(axes.flat,[
        ('Vertical RMS (m/s2)',lambda m:m['az_rms_m_s2'],None),
        ('10ms vertical peak (m/s2)',lambda m:m['az_10ms_peak_m_s2'],15),
        ('Peak wheel load / static',lambda m:max(m['wheel_load_peak_ratio_to_static']),2.5),
        ('Loaded wheel speed proxy (m/s)',lambda m:m['loaded_slip_proxy_peak_m_s'],.35)]):
        ax.bar(range(len(subset)),[fn(row['result']['metrics']) for row in subset],color=['#245b7c']+['#879cac']*(len(subset)-1))
        ax.set_xticks(range(len(labels)));ax.set_xticklabels(labels,rotation=65,ha='right',fontsize=8)
        ax.set_ylabel(label);ax.grid(axis='y',alpha=.25)
        if limit is not None:ax.axhline(limit,color='#b04436',ls='--',lw=1)
    fig.suptitle('Wide pit 10cm / requested 1km/h — joint research screen required\nChanged actual speeds; bars alone do not establish improvement or acceptance')
    fig.tight_layout();fig.savefig(base/'iteration_metrics.png',dpi=160);plt.close(fig)
    print(json.dumps(dict(new_runs=len(rows)-1,new_passes=output['passing_new_count'],full_acceptance=False)))


if __name__=='__main__':main()
