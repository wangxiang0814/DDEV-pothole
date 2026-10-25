"""Spring-seat/Jnc map from the unchanged tuned_ii_20261005 source, in SI.

SHA256496f280f0ef547b8c42ffb4e632184e511a007a8799ccd84b1ae02a8c20b52ce.
Endpoint linear extrapolation matches the native map definition. This is model
geometry, not a measured actuator stroke or validated hardware capability.
"""
import numpy as np


def seat_kinematics(jounce):
    j=np.asarray(jounce,float)
    if j.shape!=(4,) or not np.isfinite(j).all():raise ValueError('finite four-corner SI jounce required')
    knots=np.array([0,.01,.02,.03]);g=np.zeros(4);mu=np.zeros(4)
    for begin,values in ((0,[0,.0065144,.01368024,.0214977]),(2,[0,.00797,.016737,.026301])):
        table=np.asarray(values);x=j[begin:begin+2]
        index=np.clip(np.searchsorted(knots,x,side='right')-1,0,2)
        slope=(table[index+1]-table[index])/(knots[index+1]-knots[index])
        g[begin:begin+2]=table[index]+(x-knots[index])*slope;mu[begin:begin+2]=slope
    return g,mu
