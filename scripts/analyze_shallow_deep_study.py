"""Compile verified deep-pit matrices, matched deltas and finite coverage plots.

Run analyze_shallow_preview.py for each cohort first. This compiler rechecks
the run evidence and its result identity before using any derived row.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from analyze_shallow_preview import verified_run,sha


def load_verified(path):
    data=json.loads(path.read_text(encoding='utf-8'))
    plan_path=path.parent/'frozen_plan.json'
    plan=json.loads(plan_path.read_text(encoding='utf-8'));plan_sha=sha(plan_path)
    if (data['plan_sha256']!=plan_sha or data['scope']!=plan['scope'] or
            data['config_sha256']!=plan['config_sha256'] or
            sha(ROOT/plan['config_path'])!=plan['config_sha256'] or data['full_acceptance']):
        raise ValueError('cohort provenance drift: '+str(path))
    identity=lambda r:(r['case'],r['run_id'])
    expected={identity(r):r for r in plan['cases']}
    observed=[identity(r) for r in data['rows']]
    if (len(expected)!=len(plan['cases']) or len(set(observed))!=len(observed) or
            set(observed)!=set(expected) or data['count']!=len(observed)):
        raise ValueError('incomplete or duplicate cohort: '+str(path))
    for row in data['rows']:
        planned=expected[identity(row)]
        if any(row[k]!=planned[k] for k in ('case','run_id','kind','depth_m','crossing_kph')):
            raise ValueError('case classification drift: '+row['run_id'])
        run=ROOT/'runs/shallow_press'/row['case']/row['run_id']
        result,manifest=verified_run(run)
        marker=json.loads((run/'matrix_identity.json').read_text(encoding='utf-8'))
        if (marker!=dict(plan_sha256=plan_sha,args=planned['args']) or
                manifest['source_sha256']!=data['source_sha256'] or
                any(manifest['source_sha256'].get(n)!=v for n,v in plan['source_sha256'].items())):
            raise ValueError('run does not match frozen cohort: '+row['run_id'])
        failures=[k for k,v in result['criteria'].items() if not v]
        if (sha(run/'result.json')!=row['result_sha256'] or result['metrics']!=row['metrics'] or
                result['status']!=row['status'] or failures!=row['failed_criteria'] or
                result['scene']['depth_m']!=row['depth_m'] or
                result['config']['crossing_kph']!=row['crossing_kph']):
            raise ValueError('derived matrix drift: '+str(run))
        if result['acceptance_pass']:raise ValueError('unexpected full acceptance')
        for name in ('exit_progress','chassis_diagnostic','plant','capability','diagnostic_motion_pass'):
            if name in row and row[name]!=result.get(name):raise ValueError('derived '+name+' drift')
        if ('input_provenance_sha256' in row and
                row['input_provenance_sha256']!=manifest['input_provenance_sha256']):
            raise ValueError('input provenance drift')
        for name,digest in row.get('native_history_sha256',{}).items():
            if sha(run/name)!=digest:raise ValueError('native playback history drift')
    if data['passing']!=sum(r['status']=='SIMULATION_PASS' for r in data['rows']):
        raise ValueError('derived pass count drift')
    return data


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evidence',type=Path,default=ROOT/'evidence/shallow_deep_coordination_20261025')
    p.add_argument('--report',type=Path,default=ROOT/'docs/SHALLOW_DEEP_COORDINATION_20261025.md')
    a=p.parse_args();base=a.evidence
    paths={
        'original_deep':ROOT/'evidence/shallow_deep_baseline_20261025/verified_analysis.json',
        'coverage':base/'coverage/verified_analysis.json',
        'confirmation':base/'confirmation/verified_analysis.json',
        'intermediate':base/'intermediate/verified_analysis.json'}
    groups={name:load_verified(path) for name,path in paths.items()}
    for name,count in (('original_deep',12),('coverage',72),('confirmation',8),('intermediate',4)):
        if len(groups[name]['rows'])!=count:raise ValueError('unexpected study cohort size: '+name)
    coverage=groups['coverage']['rows'];confirmation=groups['confirmation']['rows']
    intermediate=groups['intermediate']['rows']
    expected_confirm={(d,v,k) for d,v in ((.10,1),(.04,3))
                      for k in ('repeat','step025','delay80','traction')}
    if ({(r['depth_m'],r['crossing_kph'],r['kind']) for r in confirmation}!=expected_confirm or
            any(r['case']!='wide' for r in confirmation)):
        raise ValueError('incomplete confirmation domain')
    if {(r['case'],r['depth_m'],r['crossing_kph']) for r in intermediate}!={
            (c,.04,v) for c in ('wide','right_track') for v in (1.5,2.5)}:
        raise ValueError('incomplete intermediate-speed domain')
    if len(coverage)!=72:raise ValueError('coverage must contain all72 cases')
    key=lambda r:(r['case'],r['depth_m'],r['crossing_kph'])
    current={key(r):r for r in coverage}
    expected={(c,d,v) for c in ('right_track','wide') for d in (.02,.04,.06,.08,.10,.12) for v in range(1,7)}
    if set(current)!=expected:raise ValueError('incomplete coverage domain')
    if {key(r) for r in groups['original_deep']['rows']}!={
            (c,d,v) for c in ('wide','right_track') for d in (.08,.10,.12) for v in (1,3)}:
        raise ValueError('incomplete original deep-pit domain')
    deltas=[]
    for previous in groups['original_deep']['rows']:
        after=current[key(previous)]
        before_result,_=verified_run(ROOT/'runs/shallow_press'/previous['case']/previous['run_id'])
        after_result,_=verified_run(ROOT/'runs/shallow_press'/after['case']/after['run_id'])
        checks=dict(config=before_result['config']==after_result['config'],
            scene=before_result['scene']==after_result['scene'],
            plant_recipe=before_result['plant']['recipe_sha256']==after_result['plant']['recipe_sha256'],
            capability=before_result['capability']==after_result['capability'])
        if not all(checks.values()):raise ValueError('unmatched comparison '+str(key(previous)))
        b=previous['metrics'];n=after['metrics']
        row=dict(case=previous['case'],depth_m=previous['depth_m'],target_kph=previous['crossing_kph'],
            before=previous['run_id'],after=after['run_id'],matched_input_checks=checks,
            attribution='COMBINED_FIX_SAME_TARGET_NOT_EXACT_SPEED_OR_SINGLE_FIX_ABLATION')
        for name in ('az_rms_m_s2','roll_peak_deg','pitch_peak_deg','loaded_slip_proxy_peak_m_s',
                     'az_10ms_peak_m_s2','crossing_mean_kph','backward_distance_m'):
            row[name]=dict(before=b[name],after=n[name],delta=n[name]-b[name])
        row['peak_load_ratio']=dict(before=max(b['wheel_load_peak_ratio_to_static']),
            after=max(n['wheel_load_peak_ratio_to_static']))
        deltas.append(row)
    failing=Counter(k for r in coverage for k in r['failed_criteria'])
    passed=[r for r in coverage if r['status']=='SIMULATION_PASS']
    record=dict(scope='FINITE_RESEARCH_COVERAGE_NOT_CONTINUOUS_ROBUSTNESS_OR_SAFETY',
        analyzer_sha256=sha(Path(__file__)),input_sha256={k:sha(v) for k,v in paths.items()},
        coverage_count=len(coverage),coverage_pass=len(passed),full_acceptance=False,
        failure_counts=dict(failing),matched_deep_comparisons=deltas,
        confirmation=[dict(case=r['case'],run_id=r['run_id'],depth_m=r['depth_m'],
            speed_kph=r['crossing_kph'],kind=r['kind'],status=r['status'],
            metrics=r['metrics'],failed_criteria=r['failed_criteria']) for r in confirmation],
        intermediate=intermediate)
    (base/'study_summary.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    with (base/'coverage_metrics.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f);writer.writerow(['case','depth_cm','target_kph','actual_mean_kph','status',
            'az_rms_m_s2','roll_deg','pitch_deg','slip_proxy_m_s','peak_load_ratio','longest_contact_loss_s',
            'backward_m','failed_criteria','run_id'])
        for r in sorted(coverage,key=key):
            m=r['metrics'];writer.writerow([r['case'],r['depth_m']*100,r['crossing_kph'],m['crossing_mean_kph'],r['status'],
                m['az_rms_m_s2'],m['roll_peak_deg'],m['pitch_peak_deg'],m['loaded_slip_proxy_peak_m_s'],
                max(m['wheel_load_peak_ratio_to_static']),max(m['longest_contact_loss_s']),m['backward_distance_m'],
                ';'.join(r['failed_criteria']),r['run_id']])
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    depths=(.02,.04,.06,.08,.10,.12);speeds=range(1,7)
    fig,axes=plt.subplots(2,3,figsize=(14,8),sharex=True,sharey=True)
    max_rms=max(r['metrics']['az_rms_m_s2'] for r in coverage)
    max_load=max(max(r['metrics']['wheel_load_peak_ratio_to_static']) for r in coverage)
    for i,(case,label) in enumerate((('right_track','Right track'),('wide','Wide pit'))):
        data=[[current[(case,d,v)] for v in speeds] for d in depths]
        grid=np.array([[r['status']=='SIMULATION_PASS' for r in row] for row in data])
        axes[i,0].imshow(grid,origin='lower',aspect='auto',vmin=0,vmax=1,cmap=ListedColormap(['#f3c7c5','#b9e2c5']))
        for y,row in enumerate(grid):
            for x,status in enumerate(row):axes[i,0].text(x,y,'P' if status else 'F',ha='center',va='center')
        for j,(field,top) in enumerate((('az_rms_m_s2',max_rms),('wheel_load_peak_ratio_to_static',max_load)),1):
            values=np.array([[max(r['metrics'][field]) if j==2 else r['metrics'][field] for r in row] for row in data])
            im=axes[i,j].imshow(values,origin='lower',aspect='auto',vmin=0,vmax=top,cmap='viridis')
            for y,row in enumerate(values):
                for x,value in enumerate(row):axes[i,j].text(x,y,'%.1f'%value,ha='center',va='center',color='white',fontsize=8)
            fig.colorbar(im,ax=axes[i,j],fraction=.046,pad=.03)
        for ax in axes[i]:
            ax.set_xticks(range(6));ax.set_xticklabels(list(speeds))
            ax.set_yticks(range(6));ax.set_yticklabels([int(d*100) for d in depths])
            ax.set_xlabel('Requested crawl speed (km/h)')
        axes[i,0].set_ylabel(label+' / depth (cm)')
    for ax,title in zip(axes[0],('Joint screen: P / F','Vertical RMS (m/s2)','Peak wheel load / static')):ax.set_title(title)
    fig.suptitle('Corrected V2 / adopted MF52 research plant / original actuation bounds\nFinite tested points; model applicability unresolved; P is not full safety acceptance',fontsize=11)
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(base/'coverage_summary.png',dpi=160);plt.close(fig)
    lines=['## 覆盖结果与交付校验','',
        '最终72项均已按完整40 s结束并核查来源，%d项通过研究联合筛查。下表仅列已测试的离散速度，不能连成连续能力区间；未通过不等于只靠行程无法通过。'%len(passed),'',
        '|深度|单侧坑通过速度km/h|宽坑通过速度km/h|','|---|---|---|']
    for depth in depths:
        cells=[]
        for case in ('right_track','wide'):
            available=[str(v) for v in speeds if current[(case,depth,v)]['status']=='SIMULATION_PASS']
            cells.append('、'.join(available) or '无')
        lines.append('|%g cm|%s|%s|'%(depth*100,*cells))
    lines+=['','![固定矩阵研究筛查和性能](../evidence/shallow_deep_coordination_20261025/coverage_summary.png)','',
        '完整指标见[CSV](../evidence/shallow_deep_coordination_20261025/coverage_metrics.csv)，匹配输入校验、旧/新深坑差值、失败统计见[机器汇总](../evidence/shallow_deep_coordination_20261025/study_summary.json)。',
        '', '重复、半步长、80 ms车身时延及仅转矩消融：','',
        '|深度/目标|对照|实际均速|轮速误差m/s|垂向RMS m/s²|轮载比|联合筛查|',
        '|---|---|---:|---:|---:|---:|---|']
    for r in confirmation:
        m=r['metrics'];lines.append('|%g cm / %g km/h|%s|%.3f|%.3f|%.3f|%.3f|%s|'%(r['depth_m']*100,
            r['crossing_kph'],r['kind'],m['crossing_mean_kph'],m['loaded_slip_proxy_peak_m_s'],m['az_rms_m_s2'],
            max(m['wheel_load_peak_ratio_to_static']),'通过' if r['status']=='SIMULATION_PASS' else '未通过'))
    lines+=['','见[确认试验工况索引](../evidence/shallow_deep_coordination_20261025/confirmation/case_index.md)。',
        '', '4 cm中间速度补测：','',
        '|坑类|目标km/h|实际均速|轮速误差m/s|联合筛查|','|---|---:|---:|---:|---|']
    for r in intermediate:
        m=r['metrics'];lines.append('|%s|%g|%.3f|%.3f|%s|'%(r['case'],r['crossing_kph'],m['crossing_mean_kph'],
            m['loaded_slip_proxy_peak_m_s'],'通过' if r['status']=='SIMULATION_PASS' else '未通过'))
    lines+=['','见[中间速度工况索引](../evidence/shallow_deep_coordination_20261025/intermediate/case_index.md)。',
        '', '研究矩阵中的PASS不替代轮胎短障碍适用性、真实底盘几何与实体执行器验证。未建立全深度/全速度通过保证，也未发布新的深度切换器。']
    text=a.report.read_text(encoding='utf-8');marker='## 覆盖结果与交付校验'
    if marker not in text:raise ValueError('report section missing')
    a.report.write_text(text[:text.index(marker)]+'\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(dict(coverage_count=72,coverage_pass=len(passed),full_acceptance=False)))


if __name__=='__main__':main()
