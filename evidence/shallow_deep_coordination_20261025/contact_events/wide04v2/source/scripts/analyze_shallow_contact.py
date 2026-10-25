"""Verify a run and export the worst loaded wheel-speed event for diagnosis.

Hub velocities use centred differences of metre positions OFFLINE. The radial
contact tangent is an approximation, not a replacement acceptance criterion.
The source run, thresholds, controller and native histories are never edited.
"""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from analyze_shallow_preview import verified_run,sha
from ddevsim.strategies.shallow_press.assessment import read_native_log
from ddevsim.strategies.shallow_press.contact_kinematics import tangent_transport


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--window-s',type=float,default=1.)
    a=p.parse_args()
    if not .05<=a.window_s<=5:p.error('window must be0.05..5s')
    result,manifest=verified_run(a.run)
    a.output.mkdir(parents=True,exist_ok=False)
    d=read_native_log(a.run/'native_1ms.csv');t=d['time_s']
    wheels=('L1','R1','L2','R2');labels=('FL','FR','RL','RR')
    matrix=lambda prefix,suffix='':np.column_stack([d['exp_'+prefix+w+suffix] for w in wheels])
    centers=np.stack([matrix(axis+'_') for axis in ('X','Y','Z')],axis=2)
    contacts=np.stack([matrix('Xctc_','i'),matrix('Yctc_','i'),matrix('Zgnd_','i')],axis=2)
    velocity=np.gradient(centers,t,axis=0)
    rim=matrix('AVy_')*np.pi/30*matrix('RRE_','i')*.001
    local=d['exp_Vx'][:,None]/3.6-d['exp_AVz'][:,None]*np.pi/180*(matrix('Y_')-d['exp_Yo'][:,None])
    proxy=rim-local
    eligible=(matrix('Fz_')>300)&(matrix('CmpT_','i')>.1)&(t[:,None]>=result['config']['settle_s'])
    score=np.where(eligible,abs(proxy),-1.)
    if not eligible.any():raise ValueError('no loaded event')
    k,i=np.unravel_index(np.argmax(score),score.shape)
    ids=np.flatnonzero(abs(t-t[k])<=a.window_s)
    tangent=np.array([tangent_transport(velocity[n],centers[n],contacts[n],np.deg2rad(d['exp_Yaw'][n])) for n in ids])[:,i]
    w=wheels[i]
    channels=dict(time_s=t[ids],body_vx_m_s=d['exp_Vx'][ids]/3.6,
        wheel_x_m=centers[ids,i,0],wheel_rim_speed_m_s=rim[ids,i],
        hub_vx_m_s=velocity[ids,i,0],hub_vz_m_s=velocity[ids,i,2],
        body_proxy_m_s=proxy[ids,i],hub_horizontal_error_m_s=rim[ids,i]-velocity[ids,i,0],
        radial_tangent_error_m_s=rim[ids,i]-tangent,
        native_kappa=d['exp_Kappa_'+w+'i'][ids],wheel_load_n=d['exp_Fz_'+w][ids],
        motor_torque_nm=d['imp_IMP_MYUSM_'+w][ids],
        ground_moment_nm=d['exp_My_WC_'+w][ids],
        active_force_n=d['imp_IMP_FS_'+w][ids],body_az_m_s2=d['exp_Az_SM'][ids]*9.80665)
    m=int(np.flatnonzero(ids==k)[0])
    event={name:float(values[m]) for name,values in channels.items()}
    rate=np.diff(channels['motor_torque_nm'])/np.diff(channels['time_s'])
    bound=result['capability']['actual_torque_rate_nm_s']
    event.update(wheel=labels[i],native_wheel=w,
        rate_saturation_fraction=float(np.mean(abs(rate)>.99*bound)),
        body_proxy_threshold_m_s=.35,radial_tangent_is_approximate=True,
        hub_velocity_method='OFFLINE_CENTRED_POSITION_DIFFERENCE_NOT_CAUSAL_FEEDBACK')
    sources=[Path(__file__),ROOT/'scripts/analyze_shallow_preview.py']
    sources += [ROOT/'src/ddevsim'/n for n in ('units.py','strategies/shallow_press/assessment.py',
        'strategies/shallow_press/contact_kinematics.py')]
    source_sha={}
    for source in sources:
        name=source.resolve().relative_to(ROOT)
        target=a.output/'source'/name;target.parent.mkdir(parents=True,exist_ok=True)
        target.write_bytes(source.read_bytes());source_sha[str(name)]=sha(source)
    record=dict(scope='CONTACT_EVENT_DIAGNOSTIC_NOT_ACCEPTANCE',run=str(a.run.resolve()),
        result_sha256=sha(a.run/'result.json'),native_csv_sha256=result['native_csv_sha256'],
        analyzer_sha256=sha(Path(__file__)),diagnostic_source_sha256=source_sha,
        event=event,criteria_unchanged=result['criteria'],
        limitations=['Radial vector is not necessarily the native tire contact-frame normal.',
          'Wheel steering/camber and contact-frame motion require native model validation.',
          'Native Kappa corroborates slip but is not a calibrated physical observation.'])
    (a.output/'event.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    with (a.output/'event.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(channels)
        writer.writerows(zip(*channels.values()))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(5,1,figsize=(10,10),sharex=True)
    tt=t[ids]-t[k]
    for n,label in [('body_proxy_m_s','Body proxy'),('radial_tangent_error_m_s','Radial tangent approx.')]:
        axes[0].plot(tt,channels[n],label=label)
    axes[0].set_ylabel('Speed error (m/s)');axes[0].legend(fontsize=8)
    for n,label in [('motor_torque_nm','Motor'),('ground_moment_nm','Ground reaction')]:axes[1].plot(tt,channels[n],label=label)
    axes[1].set_ylabel('Moment (Nm)');axes[1].legend(fontsize=8)
    axes[2].plot(tt,channels['wheel_load_n']);axes[2].set_ylabel('Wheel load (N)')
    axes[3].plot(tt,channels['active_force_n']);axes[3].set_ylabel('Active force (N)')
    axes[4].plot(tt,channels['body_az_m_s2']);axes[4].set_ylabel('Body Az (m/s2)')
    axes[4].set_xlabel('Time from worst loaded wheel-speed event (s)')
    for ax in axes:ax.grid(alpha=.25);ax.axvline(0,color='gray',ls=':')
    fig.suptitle(a.run.name+' / '+labels[i]+'\nApproximate contact diagnosis; original criteria unchanged',fontsize=10)
    fig.tight_layout();fig.savefig(a.output/'event.png',dpi=150);plt.close(fig)
    print(json.dumps(event))


if __name__=='__main__':main()
