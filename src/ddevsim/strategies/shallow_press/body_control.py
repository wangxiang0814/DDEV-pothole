"""Small-angle body-mode feedback with an explicit geometric force allocation."""
import numpy as np


def modal_feedback(x,y,heave,heave_rate,roll,pitch,roll_rate,pitch_rate,
                   height_gain,attitude_gain,velocity_gain,angular_velocity_gain=None):
    angular=velocity_gain if angular_velocity_gain is None else angular_velocity_gain
    x=np.asarray(x,float);y=np.asarray(y,float)
    state=np.asarray([heave,heave_rate,roll,pitch,roll_rate,pitch_rate,
                      height_gain,attitude_gain,velocity_gain,angular],float)
    if x.shape!=(4,) or y.shape!=(4,) or not np.isfinite(np.r_[x,y,state]).all():
        raise ValueError('finite four-corner geometry and body state required')
    matrix=np.stack((np.ones(4),y,-x))
    gram=matrix@matrix.T
    if np.linalg.cond(gram)>1e5:raise ValueError('degenerate body force geometry')
    roll_scale=float(y@y);pitch_scale=float(x@x)
    wrench=np.array([-4*(height_gain*heave+velocity_gain*heave_rate),
        -roll_scale*(attitude_gain*roll+angular*roll_rate),
        -pitch_scale*(attitude_gain*pitch+angular*pitch_rate)])
    return matrix.T@np.linalg.solve(gram,wrench)
