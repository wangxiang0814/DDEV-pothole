"""Matched idle-process A/B timing; CSV replay, not native solver timing."""
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np
import pandas as pd

root=Path.cwd();sys.path.insert(0,str(root/'src'))
from ddevsim.runtime.contracts import StrategyContext
from ddevsim.strategies.shallow_press.config import ShallowPressConfig
from ddevsim.strategies.shallow_press.event_controller import EventCrawlStrategy
from ddevsim.strategies.shallow_press.native import NativePreviewBridge
from ddevsim.strategies.shallow_press.actuation import CrawlActuation

run=root/'runs/shallow_press/wide/event_approach6_wide_d04_a60_20261027_001'
m=json.loads((run/'run_manifest.json').read_text(encoding='utf-8'));r=json.loads((run/'result.json').read_text(encoding='utf-8'))
old=run/'source/src/ddevsim/strategies/shallow_press/actuation.py'
spec=importlib.util.spec_from_file_location('ddevsim.strategies.shallow_press._before_actuation',old)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
names=m['export_names'];columns=['exp_'+n for n in names]
data=pd.read_csv(run/'native_1ms.csv',usecols=['time_s']+columns,nrows=60000)
values=data[columns].to_numpy();times=data.time_s.to_numpy();c=ShallowPressConfig(**r['config']);records=[]
for variant in ['before','after','after','before']:
    s=EventCrawlStrategy();s.reset(StrategyContext('shallow_press','tuned_ii_20261005','wide',.02,dict(scenario=r['scene'],controller=asdict(c)),{}))
    bridge=NativePreviewBridge(s,names,step_s=.001,delay_s=.04,motor_delay_s=.002)
    bridge.actuation=module.CrawlActuation(c) if variant=='before' else CrawlActuation(c)
    commands=np.empty((len(times),9));begin=time.perf_counter()
    for i,(t,row) in enumerate(zip(times,values)):commands[i]=bridge(float(t),row)
    elapsed=time.perf_counter()-begin;digest=hashlib.sha256(commands.tobytes()).hexdigest()
    records.append(dict(variant=variant,elapsed_s=elapsed,command_sha256=digest));print(json.dumps(records[-1]),flush=True)
assert len({row['command_sha256'] for row in records})==1
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
out=root/'evidence/shallow_event_crawl_20261026/performance';out.mkdir(exist_ok=True)
source=out/'sources';source.mkdir(exist_ok=True)
for label,path in [('actuation_before.py',old),('actuation_after.py',root/'src/ddevsim/strategies/shallow_press/actuation.py'),('compare_event_replay.py',Path(__file__))]:
    (source/label).write_bytes(path.read_bytes())
before=np.median([z['elapsed_s'] for z in records if z['variant']=='before']);after=np.median([z['elapsed_s'] for z in records if z['variant']=='after'])
result=dict(scope='MATCHED_OFFLINE_CSV_REPLAY_NOT_NATIVE_SOLVER_SPEED_OR_CONTROL_CAPABILITY',samples=len(times),sample_period_s=.001,
    run=str(run.relative_to(root)),native_csv_sha256=r['native_csv_sha256'],records=records,
    median_before_s=float(before),median_after_s=float(after),median_speedup=float(before/after),
    same_command_bytes=True,source_sha256={p.name:sha(p) for p in source.iterdir()})
(out/'replay.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result))
