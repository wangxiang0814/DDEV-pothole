"""Classical MIMO ARX diagnostic predictor; not a certified control plant.

The input u[k] is applied over the interval from y[k] to y[k+1]. Road nodes
include the current height. Episodes remain separate; rollout never reads
future measured outputs. This model is deliberately falsifiable on holdout.
"""
from dataclasses import dataclass
import numpy as np
from scipy.linalg import expm


def finite_matrix(value, name):
    a = np.asarray(value, dtype=float)
    if a.ndim != 2 or not np.isfinite(a).all():
        raise ValueError(name + ' requires a finite matrix')
    return a


@dataclass(frozen=True)
class DynamicModel:
    coefficients: np.ndarray
    y_center: np.ndarray
    y_scale: np.ndarray
    u_scale: np.ndarray
    road_scale: np.ndarray
    spectral_radius: float
    feature_rank: int
    feature_count: int
    condition_number: float
    ridge: float

    def as_dict(self):
        result = dict(vars(self))
        for key, value in result.items():
            if isinstance(value, np.ndarray):
                result[key] = value.tolist()
        return result


def fit_dynamic_model(episodes, ridge=1e-4):
    if not episodes or not np.isfinite(ridge) or ridge < 0:
        raise ValueError('episodes and nonnegative finite ridge required')
    checked = []
    for e in episodes:
        y, u, road = (finite_matrix(e[key], key) for key in ('y','u','road'))
        if len(y) < 4 or len(y) != len(u) or len(y) != len(road):
            raise ValueError('matching episode rows required')
        if checked and (y.shape[1],u.shape[1],road.shape[1]) != tuple(x.shape[1] for x in checked[0]):
            raise ValueError('matching episode dimensions required')
        checked.append((y,u,road))
    ys, us, roads = (np.concatenate([e[i] for e in checked]) for i in range(3))
    center = ys.mean(axis=0)
    yscale = np.maximum(ys.std(axis=0),1e-6)
    uscale = np.maximum(np.sqrt(np.mean(us**2,axis=0)),1e-6)
    rscale = np.maximum(np.sqrt(np.mean(roads**2,axis=0)),1e-6)
    features, targets = [], []
    for y,u,road in checked:
        z = (y-center)/yscale
        features.append(np.column_stack((z[1:-1],z[:-2],u[1:-1]/uscale,
                                        road[2:]/rscale,road[1:-1]/rscale,
                                        np.ones(len(y)-2))))
        targets.append(z[2:])
    f, target = np.concatenate(features), np.concatenate(targets)
    # Normalized ridge; intercept is not penalized.
    gram = f.T @ f / len(f)
    penalty = np.eye(f.shape[1]) * ridge
    penalty[-1,-1] = 0
    coefficient = np.linalg.lstsq(gram+penalty,f.T@target/len(f),rcond=1e-12)[0]
    n = ys.shape[1]
    companion = np.block([[coefficient[:n].T,coefficient[n:2*n].T],
                          [np.eye(n),np.zeros((n,n))]])
    singular = np.linalg.svd(f,compute_uv=False)
    condition = float(singular[0]/max(singular[-1],1e-300))
    return DynamicModel(coefficient,center,yscale,uscale,rscale,
                        float(max(abs(np.linalg.eigvals(companion)))),
                        int(np.linalg.matrix_rank(f)),f.shape[1],condition,float(ridge))


def rollout(model, history, future_inputs, future_road):
    history = finite_matrix(history,'history')
    inputs = finite_matrix(future_inputs,'inputs')
    road = finite_matrix(future_road,'road')
    n = len(model.y_center)
    if history.shape != (2,n) or inputs.shape[1] != len(model.u_scale):
        raise ValueError('history/input dimensions do not match model')
    if road.shape != (len(inputs)+1,len(model.road_scale)):
        raise ValueError('road requires current node plus future nodes')
    previous, current = (history-model.y_center)/model.y_scale
    predictions = []
    for k in range(len(inputs)):
        feature = np.r_[current,previous,inputs[k]/model.u_scale,
                        road[k+1]/model.road_scale,road[k]/model.road_scale,1.]
        nxt = feature @ model.coefficients
        if not np.isfinite(nxt).all():
            raise ValueError('nonfinite free-run prediction')
        predictions.append(nxt*model.y_scale+model.y_center)
        previous, current = current, nxt
    return np.asarray(predictions).reshape(len(inputs),n)


def prediction_metrics(truth,prediction,contact_truth=None):
    truth, prediction = finite_matrix(truth,'truth'),finite_matrix(prediction,'prediction')
    if truth.shape != prediction.shape or len(truth) == 0:
        raise ValueError('matching nonempty predictions required')
    error = prediction-truth
    result = {'rmse': np.sqrt(np.mean(error**2,axis=0)).tolist(),
              'max_abs': np.max(abs(error),axis=0).tolist(), 'samples': len(truth)}
    if contact_truth is not None:
        contact = np.asarray(contact_truth,dtype=bool)
        if contact.shape != truth.shape:
            raise ValueError('contact truth shape mismatch')
        result['false_support_samples'] = int(np.sum((~contact)&(prediction>=500)))
        result['lost_contact_samples'] = int(np.sum(~contact))
    return result


def propagate_state_space(a,b,c,d,initial_state,inputs,*,dt):
    a,b,c,d=(finite_matrix(value,name) for value,name in zip((a,b,c,d),'ABCD'))
    inputs=finite_matrix(inputs,'inputs')
    x=np.asarray(initial_state,dtype=float).copy()
    n,m=b.shape
    if (a.shape!=(n,n) or c.shape[1]!=n or d.shape!=(c.shape[0],m)
            or x.shape!=(n,) or inputs.shape[1]!=m or not np.isfinite(x).all()
            or not np.isfinite(dt) or dt<=0):
        raise ValueError('consistent finite state-space dimensions and timestep required')
    exact=expm(np.block([[a,b],[np.zeros((m,n+m))]])*dt)
    ad,bd=exact[:n,:n],exact[:n,n:]
    out=[]
    for u in inputs:
        x=ad@x+bd@u
        out.append(c@x+d@u)
    result=np.asarray(out).reshape(len(inputs),c.shape[0])
    if not np.isfinite(result).all():
        raise ValueError('nonfinite state-space prediction')
    return result
