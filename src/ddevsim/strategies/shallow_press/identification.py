"""SI observation features and safe parser for native local state-space matrices."""
import re
import numpy as np
from ddevsim.chassis_contact import road_surface
from .validation import WHEELS


OUTPUT_NAMES = (tuple('Jnc_'+w for w in WHEELS)+tuple('JncR_'+w for w in WHEELS)
                +tuple('Fz_'+w for w in WHEELS)+('Roll_E','Pitch','AVx','AVy','Vx')
                +tuple('AVy_'+w for w in WHEELS)+('Az_SM',))
OUTPUT_SI_FACTORS = np.array([.001]*8+[1.]*4+[np.pi/180]*4+[1/3.6]+[2*np.pi/60]*4+[9.80665])
INPUT_NAMES = tuple('IMP_MYUSM_'+w for w in WHEELS)+tuple('IMP_FS_'+w for w in WHEELS)+('IMP_STEER_SW',)


def trace_features(values,scene):
    t=np.asarray(values['observed_time_s'],dtype=float)
    if len(t)<3 or not np.allclose(np.diff(t),.02,atol=1e-6):
        raise ValueError('20 ms causal control interval trace required')
    starts=np.asarray(values['command_start_s'])
    age=starts-t
    if not np.all(age>0) or not np.allclose(age,age[0],atol=1e-7) or max(age)>.001:
        raise ValueError('command-to-observation timing mismatch')
    y=np.column_stack([values['exp_'+n] for n in OUTPUT_NAMES])*OUTPUT_SI_FACTORS
    u=np.column_stack([values['imp_'+n] for n in INPUT_NAMES])
    u[:,-1]*=np.pi/180
    xy=np.stack([np.column_stack([values['exp_X_'+w],values['exp_Y_'+w]]) for w in WHEELS],axis=1)
    ground=road_surface(xy,scene)[0]
    loads=np.column_stack([values['exp_Fz_'+w] for w in WHEELS])
    compression=np.column_stack([values['exp_CmpT_'+w+'i'] for w in WHEELS])
    contact=~((loads<100)&(compression<=.1))
    if not np.isfinite(y).all() or not np.isfinite(u).all():
        raise ValueError('nonfinite native data')
    return dict(y=y,u=u,road=ground,t=t,xy=xy,contact=contact)


def parse_native_linearization(text):
    result={}
    for key in 'ABCD':
        match=re.search(r'Matrix'+key+r'\s*=\s*zeros\(\s*(\d+)\s*,\s*(\d+)\s*\)',text)
        if not match:
            raise ValueError('missing matrix '+key)
        matrix=np.zeros(tuple(map(int,match.groups())))
        for row,col,value in re.findall(r'Matrix'+key+r'\((\d+),(\d+)\)\s*=\s*([-+0-9.eE]+)\s*;',text):
            matrix[int(row)-1,int(col)-1]=float(value)
        if not np.isfinite(matrix).all():
            raise ValueError('nonfinite native matrix')
        result[key]=matrix
    n=result['A'].shape[0]
    if (result['A'].shape!=(n,n) or result['B'].shape[0]!=n or result['C'].shape[1]!=n
            or result['D'].shape!=(result['C'].shape[0],result['B'].shape[1])):
        raise ValueError('inconsistent native matrix dimensions')
    for label,key in [('SV','state_names'),('IN','input_names'),('OUT','output_names')]:
        names=re.findall(r'%\s*'+label+r'\(\d+\)\s*=\s*(\w+)',text)
        result[key]=names
    return result


def prediction_gate(metrics,*,false_support_samples):
    rms=np.asarray(metrics['rmse'])
    peak=np.asarray(metrics['max_abs'])
    if rms.shape!=(22,) or peak.shape!=(22,) or not np.isfinite(rms).all() or not np.isfinite(peak).all():
        raise ValueError('finite 22 output metrics required')
    def within(indices,a,b):
        return bool(np.max(rms[indices])<=a and np.max(peak[indices])<=b)
    result=dict(jounce=within(slice(0,4),.005,.015),
                attitude=within(slice(12,14),np.deg2rad(.5),np.deg2rad(1.5)),
                load=within(slice(8,12),250,750),speed=within(slice(16,17),.05,.15),
                contact=false_support_samples==0)
    result['joint_mpc_model']=all(result.values())
    return result


def qualify_pit_prediction(metrics,edge_metrics,*,false_support_samples,rear_cleared):
    result=prediction_gate(metrics,false_support_samples=false_support_samples)
    result['pit_edges']=(False if edge_metrics is None else
                        prediction_gate(edge_metrics,false_support_samples=0)['joint_mpc_model'])
    result['rear_completed']=bool(rear_cleared)
    result['joint_mpc_model']=all(value for key,value in result.items() if key!='joint_mpc_model')
    return result
