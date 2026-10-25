"""Causal wheel-hub motion and a radial contact-tangent approximation.

Uses verified metre positions, not the legacy ambiguously documented Vz_Wc.
The radial vector is an approximation of the contact frame at a sharp edge;
it is not a replacement for native tire-frame validation or tire calibration.
"""
import math
import numpy as np

WHEELS=('L1','R1','L2','R2')
HUB_REQUIRED=tuple(p+w+s for p,s in (('X_',''),('Y_',''),('Z_',''),('Xctc_','i'),('Yctc_','i'),('Zgnd_','i')) for w in WHEELS)


def tangent_transport(velocity,centers,contacts,yaw):
    velocity=np.asarray(velocity,float);centers=np.asarray(centers,float);contacts=np.asarray(contacts,float)
    if any(a.shape!=(4,3) for a in (velocity,centers,contacts)) or not np.isfinite(np.r_[velocity.ravel(),centers.ravel(),contacts.ravel(),yaw]).all():
        raise ValueError('finite four-wheel Cartesian geometry required')
    forward=np.array([math.cos(yaw),math.sin(yaw),0.])
    radial=centers-contacts;length=np.linalg.norm(radial,axis=1)
    valid=(length>.08)&(length<.6)
    normal=np.tile([0.,0.,1.],(4,1))
    normal[valid]=radial[valid]/length[valid,None]
    tangent=forward-normal*(normal@forward)[:,None]
    norm=np.linalg.norm(tangent,axis=1)
    tangent=np.where((norm>1e-4)[:,None],tangent/np.maximum(norm,1e-4)[:,None],forward)
    return np.einsum('ij,ij->i',velocity,tangent)


class WheelMotion:
    def __init__(self,filter_s=.004):
        if not math.isfinite(filter_s) or not 0<=filter_s<=.05:raise ValueError('invalid wheel velocity filter')
        self.filter_s=filter_s;self.reset()

    def reset(self):
        self.previous=None;self.stamp=None;self.velocity=None;self.transport=np.zeros(4)

    def observe(self,stamp,feedback):
        centers=np.array([[feedback[a+'_'+w] for a in ('X','Y','Z')] for w in WHEELS])
        contacts=np.array([[feedback['Xctc_'+w+'i'],feedback['Yctc_'+w+'i'],feedback['Zgnd_'+w+'i']] for w in WHEELS])
        dt=0. if self.stamp is None else stamp-self.stamp
        if self.stamp is not None and dt<=1e-10:return self.transport.copy()
        yaw=feedback['Yaw'];forward=np.array([math.cos(yaw),math.sin(yaw),0.])
        if self.previous is None or not 0<dt<=.02:
            self.velocity=np.tile(feedback['Vx']*forward,(4,1))
        else:
            derivative=(centers-self.previous)/dt
            alpha=1. if self.filter_s==0 else 1-math.exp(-dt/self.filter_s)
            self.velocity+=alpha*(derivative-self.velocity)
        self.previous=centers.copy();self.stamp=stamp
        self.transport=tangent_transport(self.velocity,centers,contacts,yaw)
        return self.transport.copy()
