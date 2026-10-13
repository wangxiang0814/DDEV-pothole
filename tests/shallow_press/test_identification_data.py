import numpy as np
import pytest

from ddevsim.strategies.shallow_press.identification import (
    trace_features,parse_native_linearization,OUTPUT_NAMES)


def test_trace_units_and_interval_alignment_are_explicit():
    dtype=[('observed_time_s',float),('command_start_s',float)]
    wheels=('L1','R1','L2','R2')
    exports=[*('Jnc_'+w for w in wheels),*('JncR_'+w for w in wheels),
             *('Fz_'+w for w in wheels),'Roll_E','Pitch','AVx','AVy','Vx',
             *('AVy_'+w for w in wheels),'Az_SM',
             *('X_'+w for w in wheels),*('Y_'+w for w in wheels),
             *('CmpT_'+w+'i' for w in wheels)]
    dtype += [('exp_'+n,float) for n in exports]
    dtype += [('imp_IMP_MYUSM_'+w,float) for w in wheels]
    dtype += [('imp_IMP_FS_'+w,float) for w in wheels]+[('imp_IMP_STEER_SW',float)]
    values=np.zeros(3,dtype=dtype)
    values['observed_time_s']=[0,.02,.04]
    values['command_start_s']=[.0005,.0205,.0405]
    values['exp_Jnc_L1']=100
    values['exp_Vx']=3.6
    values['exp_Roll_E']=180
    values['exp_AVy_L1']=60
    values['exp_Az_SM']=1
    values['imp_IMP_FS_L1']=[100,200,300]
    for w in wheels:
        values['exp_X_'+w]=98
        values['exp_Y_'+w]=.63
        values['exp_Fz_'+w]=1000
        values['exp_CmpT_'+w+'i']=1
    scene=dict(start_station_m=100,length_m=.8,width_m=1.8,depth_m=.04,center_y_m=0,edge_transition_m=.05)
    e=trace_features(values,scene)
    assert e['y'][0,OUTPUT_NAMES.index('Jnc_L1')] == pytest.approx(.1)
    assert e['y'][0,OUTPUT_NAMES.index('Roll_E')] == pytest.approx(np.pi)
    assert e['y'][0,OUTPUT_NAMES.index('Vx')] == 1
    assert e['y'][0,OUTPUT_NAMES.index('AVy_L1')] == pytest.approx(2*np.pi)
    assert e['y'][0,OUTPUT_NAMES.index('Az_SM')] == pytest.approx(9.80665)
    assert e['u'][:,4].tolist()==[100,200,300]
    assert np.all(e['road']==0)


def test_native_matrix_parser_reads_one_based_sparse_assignments_without_exec():
    text='''% SV(1) = SV_JNC_L1: state
% IN(1) = IMP_FS_L1; input
% OUT(1) = Jnc_L1; output
MatrixA = zeros(2, 2);
MatrixA(1,2) = 1;
MatrixA(2,1) = -2e1;
MatrixB = zeros(2,1);
MatrixB(2,1) = .5;
MatrixC = zeros(1,2);
MatrixC(1,1) = 1;
MatrixD = zeros(1,1);
'''
    result=parse_native_linearization(text)
    assert result['A'].tolist()==[[0,1],[-20,0]]
    assert result['B'].tolist()==[[0],[.5]]
    assert result['state_names'][0]=='SV_JNC_L1'


def test_bad_matrix_shape_cannot_be_published_as_native_model():
    with pytest.raises(ValueError,match='dimensions'):
        parse_native_linearization('MatrixA = zeros(2,3); MatrixB = zeros(2,1); MatrixC = zeros(1,2); MatrixD = zeros(1,1);')
