import numpy as np
import pytest

from ddevsim.strategies.shallow_press.prediction import (
    fit_dynamic_model, rollout, prediction_metrics,propagate_state_space)


def known_episode(seed, count=900):
    rng = np.random.RandomState(seed)
    u = rng.normal(size=(count, 1))
    road = rng.normal(size=(count, 1))
    y = np.zeros((count, 1))
    for k in range(1, count-1):
        y[k+1] = .75*y[k] + .125*y[k-1] + 2*u[k] + .5*road[k+1] - .25*road[k] + .2
    return {'y': y, 'u': u, 'road': road}


def test_identified_predictor_free_runs_on_unseen_inputs():
    model = fit_dynamic_model([known_episode(1), known_episode(2)], ridge=1e-10)
    test = known_episode(21)
    k = 100
    predicted = rollout(model, test['y'][k-1:k+1], test['u'][k:k+20], test['road'][k:k+21])
    np.testing.assert_allclose(predicted, test['y'][k+1:k+21], atol=1e-6)
    assert model.spectral_radius < 1


def test_rollout_uses_applied_interval_input_without_one_tick_shift():
    model = fit_dynamic_model([known_episode(4)], ridge=1e-10)
    # At zero history/ground, a command of 3 produces y_next = 6 + .2.
    result = rollout(model, np.zeros((2,1)), np.array([[3.], [0.]]), np.zeros((3,1)))
    assert result[0,0] == pytest.approx(6.2, abs=1e-6)
    assert result[1,0] == pytest.approx(4.85, abs=1e-6)


def test_episodes_are_not_joined_into_artificial_state_transitions():
    a, b = known_episode(1), known_episode(2)
    b['y'][0:2] = 1000
    for k in range(1,len(b['y'])-1):
        b['y'][k+1] = .75*b['y'][k] + .125*b['y'][k-1] + 2*b['u'][k] + .5*b['road'][k+1] - .25*b['road'][k] + .2
    model = fit_dynamic_model([a,b], ridge=1e-10)
    result = rollout(model, np.zeros((2,1)), np.zeros((1,1)), np.zeros((2,1)))
    assert result[0,0] == pytest.approx(.2, abs=1e-4)


def test_nonfinite_prediction_is_rejected_not_excluded_from_metrics():
    with pytest.raises(ValueError, match='finite'):
        prediction_metrics(np.zeros((2,1)), np.array([[0.],[np.nan]]))


def test_false_support_is_counted_even_when_mean_load_error_is_small():
    truth = np.zeros((100,4))
    pred = truth.copy()
    pred[0,0] = 600
    metrics = prediction_metrics(truth,pred,contact_truth=np.zeros((100,4),dtype=bool))
    assert metrics['false_support_samples'] == 1
    assert metrics['rmse'][0] == pytest.approx(60)
    assert metrics['max_abs'][0] == 600


def test_future_road_requires_current_height_and_every_prediction_node():
    model = fit_dynamic_model([known_episode(1)], ridge=1e-8)
    with pytest.raises(ValueError, match='road'):
        rollout(model,np.zeros((2,1)),np.zeros((10,1)),np.zeros((10,1)))


def test_native_state_space_uses_exact_zero_order_hold_and_direct_term():
    result=propagate_state_space(np.array([[-2.]]),np.array([[1.]]),
        np.array([[2.]]),np.array([[.1]]),np.zeros(1),np.ones((2,1)),dt=.1)
    assert result[0,0]==pytest.approx(1-np.exp(-.2)+.1)
    assert result[1,0]==pytest.approx(1-np.exp(-.4)+.1)
