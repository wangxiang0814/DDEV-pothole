import sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from run_feedback_fault_trial import TrialController


def test_moving_fault_triggers_only_at_selected_phase_and_geometry():
    c=object.__new__(TrialController)
    c.fault_time_s=None;c.rear_started=True;c.phase='RR_CRAWL';c.pit_fraction=.2
    c.rear=SimpleNamespace(mode='RR_HOLD',scenario={'start_station_m':101.,'length_m':.8},mode_start_s=0.)
    c.trigger(10.,{'X_R2':101.3});assert c.fault_time_s is None
    c.rear.mode='RR_CRAWL'
    c.trigger(10.,{'X_R2':101.1});assert c.fault_time_s is None
    c.trigger(11.,{'X_R2':101.3});assert c.fault_time_s==11.
    c.trigger(12.,{'X_R2':101.5});assert c.fault_time_s==11.
