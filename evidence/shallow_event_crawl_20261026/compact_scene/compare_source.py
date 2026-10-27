from pathlib import Path
import json,hashlib
import pandas as pd
r=Path.cwd();out=r/'evidence/shallow_event_crawl_20261026/compact_scene';rows=[]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for speed,old,new,d0,d1 in ((3,'event_spatial_descent_wide_d04_a30_20261027_001','event_compact_wide_d04_a30_s08_20261027_001',12,8),(6,'event_execution_wide_d04_a60_20261027_001','event_compact_wide_d04_a60_s22_20261027_001',30,22)):
 pair=[]
 for name in (old,new):
  p=r/'runs/shallow_press/wide'/name;result=json.loads((p/'result.json').read_text(encoding='utf-8'))
  d=pd.read_csv(p/'native_1ms.csv',usecols=['time_s','exp_Vx','exp_X_R1'],float_precision='round_trip')
  prefix=d.exp_X_R1<result['scene']['start_station_m']-.5
  diag=json.loads((p/'control_diagnostics.json').read_text(encoding='utf-8'))
  pair.append(dict(run_id=name,result_sha256=sha(p/'result.json'),config=result['config'],criteria=result['criteria'],metrics=result['metrics'],actual_approach_peak_kph=float(d.exp_Vx[prefix].max()),front_entry_s=result['metrics']['event_raw_crossings'][0]['entered_s'],completed_s=next((q['time_s'] for q in diag if q['phase']=='COMPLETE'),None)))
 same=pair[0]['config']==pair[1]['config']
 assert same,'controller settings drift'
 assert all(pair[1]['criteria'].values()) and pair[1]['completed_s'] is not None
 assert abs(pair[1]['actual_approach_peak_kph']-speed)<.15
 rows.append(dict(approach_kph=speed,old_start_distance_m=d0,new_start_distance_m=d1,identical_controller_settings=same,front_entry_time_saved_s=pair[0]['front_entry_s']-pair[1]['front_entry_s'],before=pair[0],after=pair[1]))
(out/'comparison.json').write_text(json.dumps(dict(scope='SHORTER_SCENE_VALIDATION_NOT_A_NEW_CONTROLLER',rows=rows),indent=2)+'\n',encoding='utf-8')
(out/'compare_source.py').write_bytes(Path(__file__).read_bytes())
print(json.dumps([dict(speed=row['approach_kph'],start_m=row['new_start_distance_m'],peak=row['after']['actual_approach_peak_kph'],rms=row['after']['metrics']['az_rms_m_s2'],slip=row['after']['metrics']['loaded_slip_proxy_peak_m_s'],complete_s=row['after']['completed_s'],time_saved=row['front_entry_time_saved_s']) for row in rows]))
