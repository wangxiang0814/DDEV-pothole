"""Check full output contact points against the actual callback plane."""
import json
from pathlib import Path
import sys
import numpy as np
import pandas as pd
root=Path.cwd();sys.path.insert(0,str(root/'scripts'))
from analyze_shallow_preview import verified_run,sha

run=root/'runs/shallow_press/wide/event_callback_circle_wide_d10_a30_20261027_001'
r,m=verified_run(run);wheels=('L1','R1','L2','R2')
cols=['time_s']+[s+w+t for w in wheels for s,t in [('exp_Xctc_','i'),('exp_Yctc_','i'),('exp_Zgnd_','i'),('exp_Fz_',''),('exp_CmpT_','i')]]
native=pd.read_csv(run/'native_1ms.csv',usecols=cols);native['key']=np.rint(native.time_s*1e6).astype(np.int64)
callback=pd.read_csv(run/'contact_callback.csv');callback['key']=np.rint(callback.command_time_s*1e6).astype(np.int64)
rows=[]
for i,w in enumerate(wheels):
    q=callback[callback.nearest_wheel==i]
    if q.key.duplicated().any():raise ValueError('ambiguous per-wheel plane')
    d=native.merge(q,on='key',how='inner',validate='one_to_one')
    loaded=(d['exp_Fz_'+w]>300)&(d['exp_CmpT_'+w+'i']>.1)
    expected=d.plane_z_at_hub_m+d.plane_dzdx*(d['exp_Xctc_'+w+'i']-d.hub_x_m)+d.plane_dzdy*(d['exp_Yctc_'+w+'i']-d.hub_y_m)
    residual=d['exp_Zgnd_'+w+'i']-expected
    query_expected=d.plane_z_at_hub_m+d.plane_dzdx*(d.query_x_m-d.hub_x_m)+d.plane_dzdy*(d.query_y_m-d.hub_y_m)
    error=abs(residual[loaded]);k=error.idxmax()
    rows.append(dict(wheel=w,merged_rows=len(d),loaded_rows=int(loaded.sum()),
        loaded_contact_plane_peak_residual_m=float(error.max()),loaded_p99_residual_m=float(error.quantile(.99)),
        actual_query_plane_peak_residual_m=float(abs(d.returned_z_m-query_expected).max()),
        worst_time_s=float(d.loc[k,'time_s']),worst_expected_z_m=float(expected[k]),worst_native_z_m=float(d.loc[k,'exp_Zgnd_'+w+'i'])))
out=root/'evidence/shallow_event_crawl_20261026/callback_full';source=out/'check_callback_plane.py';source.write_bytes(Path(__file__).read_bytes())
record=dict(scope='GEOMETRIC_INTERFACE_CHECK_NOT_TIRE_PHYSICAL_CALIBRATION',run=str(run.relative_to(root)),
    result_sha256=sha(run/'result.json'),callback_summary_sha256=sha(run/'contact_callback.json'),
    checker_sha256=sha(source),time_match_tolerance_s=1e-6,rows=rows,
    millimetre_plane_gate=all(x['loaded_contact_plane_peak_residual_m']<.001 for x in rows))
(out/'plane_check.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8');print(json.dumps(record))
