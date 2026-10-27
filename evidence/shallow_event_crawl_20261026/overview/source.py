from pathlib import Path
import json,hashlib
import numpy as np,pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path.cwd();out=r/'evidence/shallow_event_crawl_20261026/overview';out.mkdir(exist_ok=True)
runs=[('event_spatial_descent_wide_d04_a30_20261027_001','Wide pit 4 cm / approach 3 km/h'),('event_spatial_descent_wide_d10_a30_20261027_001','Wide pit 10 cm / approach 3 km/h')]
cols=['time_s','exp_Vx','exp_Az_SM','exp_Roll_E','exp_Pitch']+['exp_'+a+'_'+w for a in ('X','Fz') for w in ('R1','R2')]+['imp_IMP_'+a+'_'+w for a in ('MYUSM','FS') for w in ('R1','R2')]
fig,axes=plt.subplots(6,2,figsize=(13,13),sharex='col');records=[]
for col,(name,title) in enumerate(runs):
 p=r/'runs/shallow_press/wide'/name;d=pd.read_csv(p/'native_1ms.csv',usecols=cols,float_precision='round_trip').to_records(index=False);result=json.loads((p/'result.json').read_text(encoding='utf-8'));diag=json.loads((p/'control_diagnostics.json').read_text(encoding='utf-8'))
 enter=result['metrics']['event_raw_crossings'][0]['entered_s'];rear=result['metrics']['event_raw_crossings'][-1]['crossed_s'];mask=(d.time_s>=enter-2)&(d.time_s<=rear+3);tt=d.time_s[mask]-enter
 ref=np.interp(d.time_s,[q['time_s'] for q in diag],[q['speed_reference_m_s']*3.6 for q in diag])
 axes[0,col].plot(tt,d.exp_Vx[mask],label='Actual');axes[0,col].plot(tt,ref[mask],label='Variable reference',ls='--',color='gray')
 axes[0,col].set_title(title+'\n'+result['status']);axes[0,col].legend(fontsize=8)
 for w,label in [('R1','FR'),('R2','RR')]:
  axes[1,col].plot(tt,d['imp_IMP_MYUSM_'+w][mask],label=label)
  axes[2,col].plot(tt,d['imp_IMP_FS_'+w][mask],label=label)
  axes[3,col].plot(tt,d['exp_Fz_'+w][mask],label=label)
 axes[4,col].plot(tt,d.exp_Roll_E[mask],label='Roll');axes[4,col].plot(tt,d.exp_Pitch[mask],label='Pitch')
 axes[5,col].plot(tt,d.exp_Az_SM[mask]*9.80665)
 for row,label in enumerate(('Speed (km/h)','Motor torque (Nm)','Active force (N)','Wheel load (N)','Body angle (deg)','Body Az (m/s2)')):
  axes[row,col].set_ylabel(label);axes[row,col].grid(alpha=.25)
  axes[row,col].axvline(0,color='black',ls=':',alpha=.4)
  if row in (1,2,3,4):axes[row,col].legend(fontsize=7,loc='upper right')
 axes[5,col].set_xlabel('Time from actual front-axle pit entry (s)')
 records.append(dict(run_id=name,result_sha256=hashlib.sha256((p/'result.json').read_bytes()).hexdigest(),native_csv_sha256=result['native_csv_sha256'],front_entry_s=enter,rear_exit_s=rear,criteria=result['criteria']))
fig.suptitle('Same event controller, MF52/3114 baseline, original actuator limits\nFull native runs; focused crossing views; not real-vehicle safety validation')
fig.tight_layout(rect=(0,0,1,.95));fig.savefig(out/'wide_4cm_10cm.png',dpi=140);plt.close(fig)
(out/'source.py').write_bytes(Path(__file__).read_bytes())
(out/'provenance.json').write_text(json.dumps(dict(scope='MATCHED_CONFIG_DEPTH_DIAGNOSTIC',rows=records,script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()),indent=2)+'\n',encoding='utf-8')
print('overview saved')
