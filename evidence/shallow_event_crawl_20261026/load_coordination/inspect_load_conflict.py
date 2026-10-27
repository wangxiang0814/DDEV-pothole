from pathlib import Path
import json
import numpy as np,pandas as pd
r=Path('.');p=r/'runs/shallow_press/wide/event_crest_release_wide_d10_a30_20261027_001'
d=pd.read_csv(p/'control_intervals.csv',usecols=['observed_time_s','exp_X_R1','exp_X_R2','exp_Fz_R1','exp_Fz_R2','diag_fast_load_n_R1','diag_fast_load_n_R2','imp_IMP_FS_R1','imp_IMP_FS_R2'])
q=json.loads((p/'control_diagnostics.json').read_text(encoding='utf-8'));t=d.observed_time_s.to_numpy();ix=np.maximum(0,np.searchsorted([a['time_s'] for a in q],t,side='right')-1)
out=[]
for wi,w in ((1,'R1'),(3,'R2')):
 target=np.array([q[k]['target_force_n'][wi] for k in ix]);fast=d['diag_fast_load_n_'+w].to_numpy();x=d['exp_X_'+w].to_numpy()
 climbing=(x>101.65)&(x<101.95);opposed=climbing&(target<-200)&(fast>200)
 out.append(dict(wheel=w,climbing_samples=int(climbing.sum()),opposed_samples=int(opposed.sum()),mean_body_force_when_opposed_n=float(np.mean(target[opposed])),mean_fast_force_when_opposed_n=float(np.mean(fast[opposed])),max_fast_force_when_opposed_n=float(np.max(fast[opposed])) if opposed.any() else None))
outdir=r/'evidence/shallow_event_crawl_20261026/load_coordination';outdir.mkdir(exist_ok=True)
(outdir/'motivation.json').write_text(json.dumps(dict(scope='OFFLINE_FORCE_LOOP_OPPOSITION_NOT_PROOF_OF_CAUSAL_BENEFIT',run=str(p),result_sha256=__import__('hashlib').sha256((p/'result.json').read_bytes()).hexdigest(),rows=out),indent=2)+'\n',encoding='utf-8')
print(json.dumps(out))
