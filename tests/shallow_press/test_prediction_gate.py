import numpy as np
from ddevsim.strategies.shallow_press.identification import prediction_gate,qualify_pit_prediction


def test_load_failure_cannot_be_hidden_by_passing_attitude_and_travel():
    rms=np.zeros(22)
    maximum=np.zeros(22)
    rms[8]=251
    result=prediction_gate({'rmse':rms.tolist(),'max_abs':maximum.tolist()},false_support_samples=0)
    assert result['jounce']
    assert result['attitude']
    assert not result['load']
    assert not result['joint_mpc_model']


def test_predicted_support_during_loss_prevents_joint_mpc_release():
    result=prediction_gate({'rmse':[0]*22,'max_abs':[0]*22},false_support_samples=1)
    assert result['load']
    assert not result['contact']
    assert not result['joint_mpc_model']


def test_good_average_cannot_hide_edge_rmse_failure():
    full={'rmse':[0]*22,'max_abs':[0]*22}
    edge={'rmse':[0]*22,'max_abs':[0]*22}
    edge['rmse'][0]=.006
    edge['max_abs'][0]=.01
    result=qualify_pit_prediction(full,edge,false_support_samples=0,rear_cleared=True)
    assert not result['pit_edges']
    assert not result['joint_mpc_model']


def test_partial_passage_cannot_qualify_pit_model():
    metrics={'rmse':[0]*22,'max_abs':[0]*22}
    result=qualify_pit_prediction(metrics,metrics,false_support_samples=0,rear_cleared=False)
    assert not result['rear_completed']
    assert not result['joint_mpc_model']
