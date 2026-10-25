"""Verify original-PAC52 contact comparisons separately from the MF52 baseline."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'));sys.path.insert(0,str(ROOT/'src'))
from analyze_shallow_preview import verified_run,source_digest_map,sha
from ddevsim.strategies.shallow_press.assessment import read_native_log


def main():
    base=Path(__file__).resolve().parent
    plan=json.loads((base/'frozen_plan.json').read_text(encoding='utf-8'));rows=[];logs=[]
    if sha(ROOT/plan['config_path'])!=plan['config_sha256']:raise ValueError('config drift')
    for case in plan['cases']:
        run=ROOT/'runs/shallow_press'/case['case']/case['run_id'];result,m=verified_run(run)
        marker=json.loads((run/'matrix_identity.json').read_text(encoding='utf-8'))
        if marker!=dict(plan_sha256=sha(base/'frozen_plan.json'),args=case['args']):raise ValueError('plan binding')
        sources=source_digest_map(m['source_sha256'])
        if sources!=source_digest_map(plan['source_sha256']):raise ValueError('source binding')
        if result['status']!='MODEL_DIAGNOSTIC_ONLY' or result['simulation_pass'] or result['acceptance_pass']:raise ValueError('diagnostic identity')
        if result['model_diagnostic_motion_pass']!=all(result['criteria'].values()):raise ValueError('screen mismatch')
        history={str(p.relative_to(run)):sha(p) for p in (run/'model/output').glob('*.vs*')}
        if not history:raise ValueError('missing native playback')
        rows.append(dict(run_id=case['run_id'],result=result,result_sha256=sha(run/'result.json'),
            native_history_sha256=history,source_sha256=m['source_sha256']))
        logs.append(read_native_log(run/'native_1ms.csv'))
    archive=base/'executed_source.zip'
    first=ROOT/'runs/shallow_press'/plan['cases'][0]['case']/plan['cases'][0]['run_id']
    if not archive.exists():
        with zipfile.ZipFile(archive,'x',compression=zipfile.ZIP_DEFLATED) as z:
            for name in sorted(sources):z.write(first/'source'/name,name)
    with zipfile.ZipFile(archive) as z:
        for name,h in sources.items():
            if hashlib.sha256(z.read(name)).hexdigest()!=h:raise ValueError('source archive')
    a,b=logs[:2]
    mask=(a['time_s']>=5)&(a['exp_X_L1']<rows[0]['result']['scene']['start_station_m']-.4)
    flat={n:float(np.max(abs(a[n][mask]-b[n][mask]))) for n in ['exp_Vx','exp_Zo','exp_Fz_L1','exp_Fz_R1','exp_Fz_L2','exp_Fz_R2']}
    manuals=[]
    for path,pages in [('F:/TruckSim2019/TruckSim2019.0_Prog/Help/Screens/Tire_Models.pdf',[48,49,57,58]),
                       ('F:/TruckSim2019/TruckSim2019.0_Prog/Help/Manuals/MFTyre-MFSwift_Help.pdf',[24,26,28,29])]:
        manuals.append(dict(path=path,sha256=sha(Path(path)),pages=pages))
    output=dict(scope=plan['scope'],count=len(rows),adopted=False,baseline_changed=False,full_acceptance=False,
        compiler_sha256=sha(Path(__file__)),source_archive_sha256=sha(archive),
        manual_sources=manuals,flat_rolling_max_abs_difference_native_units=flat,
        flat_check_scope='OriginalPAC52 point vs150mm average before pit; not proof of obstacle contact validity',rows=rows)
    (base/'verified_contact_comparison.json').write_text(json.dumps(output,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(count=len(rows),model_motion_passes=sum(r['result']['model_diagnostic_motion_pass'] for r in rows),adopted=False,flat=flat)))


if __name__=='__main__':main()
