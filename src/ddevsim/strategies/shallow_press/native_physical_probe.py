"""Shallow-only native instrumentation; no modification of the public contract."""
import ctypes
from pathlib import Path
import re
import numpy as np
from .identification import INPUT_NAMES,parse_native_linearization
from .validation import ValidationExcitation,WHEELS

ROAD_NAMES=tuple('IMP_'+p+'_'+w+'I' for p in ('ZGND','DZDX','DZDY') for w in WHEELS)


def validate_reference_matrices(model,matrix_text):
    native=parse_native_linearization(matrix_text)
    for key in 'ABCD':
        if not np.array_equal(np.asarray(model[key]),native[key]):
            raise ValueError('embedded matrix diverges from native file: '+key)
    for key in ('state_names','input_names','output_names'):
        if model[key]!=native[key]:
            raise ValueError('embedded matrix names diverge from native file: '+key)


def instrument_model(text,simtext,exports):
    if not re.search(r'\nEND\s*$',text):
        raise ValueError('final END required')
    if any(name in text for name in ROAD_NAMES):
        raise ValueError('road instrumentation already present')
    ports,count=re.subn(r'(?m)^PORTS_IMP 9$', 'PORTS_IMP 21',simtext)
    if count!=1:
        raise ValueError('nine-channel source model required')
    lines=['IMPORT '+n+' Add 0.0' for n in ROAD_NAMES]
    lines+=['OPT_LINEARIZATION 0']
    lines+=['LINEAR_CONTROL '+n for n in INPUT_NAMES+ROAD_NAMES]
    lines+=['LINEAR_OUTPUT EXP_'+n for n in exports]
    return re.sub(r'\nEND\s*$', '\n'+'\n'.join(lines)+'\nEND\n',text),ports,ROAD_NAMES


class NativePhysicalProbe:
    """Records full native states (oracle initialization, not sensor estimates)."""
    def __init__(self,names,*,speed_kph,end_station_m,state_names,dll_path,model_dir,extract=False,linearize_at_s=5.):
        self.base=ValidationExcitation(names,speed_kph=speed_kph,seed=917,
            excite=False,end_station_m=end_station_m)
        self.state_names=state_names
        self.dll_path=dll_path
        self.model_dir=Path(model_dir)
        self.extract=extract
        self.linearize_at_s=float(linearize_at_s)
        self.dll=None
        self.rows=[]
        self.reference=None
        self.previous_state=None
        self.previous_time=None
        self.pointers=None

    def __call__(self,t,exports):
        if self.dll is None:
            self.dll=ctypes.cdll.LoadLibrary(str(self.dll_path))
            self.dll.vs_get_var_ptr.argtypes=[ctypes.c_char_p]
            self.dll.vs_get_var_ptr.restype=ctypes.POINTER(ctypes.c_double)
            self.dll.vs_statement.argtypes=[ctypes.c_char_p,ctypes.c_char_p,ctypes.c_int]
            self.dll.vs_statement.restype=ctypes.c_int
            self.pointers=[self.dll.vs_get_var_ptr(n.encode('ascii')) for n in self.state_names]
            if not all(self.pointers):
                raise ValueError('unavailable native state pointer')
        state=np.array([p.contents.value for p in self.pointers])
        observed=t-.0005
        derivative=(np.zeros_like(state) if self.previous_state is None else
                    (state-self.previous_state)/(observed-self.previous_time))
        self.previous_state=state
        self.previous_time=observed
        count=len(self.base.records)
        command=self.base(t,exports)
        if len(self.base.records)>count:
            row=dict(self.base.records[-1])
            row.update({'sv_'+n:float(v) for n,v in zip(self.state_names,state)})
            row.update({'fdot_'+n:float(v) for n,v in zip(self.state_names,derivative)})
            self.rows.append(row)
            if self.extract and self.reference is None and observed>=self.linearize_at_s-1e-8:
                path=self.model_dir/'output/physical_linearization.m'
                # Event extraction is read at the following observation.
                if not path.exists():
                    return command+(0.,)*12
                parsed=parse_native_linearization(path.read_text(encoding='utf-8'))
                self.reference=dict(state=state.tolist(),f0=derivative.tolist(),observed_time_s=observed,
                    native_states_are_oracle=True,
                    model={k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in parsed.items()})
        return command+(0.,)*12
