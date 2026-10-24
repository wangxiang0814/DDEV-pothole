"""Independent MF-Tyre 5.2 comparison; never mutates the shared vehicle.

Sources: installed Tire Models (Dec2018) pp58,66; MF-Tyre/MF-Swift6.2
manual pp34-35 (FITTYP6),39-46 (property-file coefficient groups).
Coefficient transfer does not establish tire/vehicle equivalence or validation.
"""
import re


def research_baseline_metadata(record):
    """User adoption on2026-10-24 is scoped to expert-two research only.

    Coefficients and assumptions remain exactly those of the comparison;
    adoption neither establishes physical equivalence nor real safety.
    """
    return dict(record,adopted=True,baseline_id='expert2_mf52_research_20261024',
        adoption_scope='EXPERT2_RESEARCH_ONLY',adoption_date='2026-10-24',
        adoption_basis='USER_SELECTED_AFTER_22_CASE_COMPARISON',
        real_vehicle_validated=False)


def make_mf52_tir(source):
    blocks=re.findall(r'BEGIN_PACEJKA_52_DATA\n(.*?)END_PACEJKA_52_DATA',source,re.S)
    if len(blocks)!=4 or len(set(blocks))!=1:raise ValueError('four identical legacy tires required')
    data={k:float(v) for k,v in re.findall(r'P52_(\w+)\s+([-+0-9.eE]+)',blocks[0])}
    sections={
        'MODEL':{'FITTYP':6,'TYRESIDE':"'Left'",'LONGVL':data['LONGVL'],'VXLOW':.2,
                 'ROAD_INCREMENT':.01,'ROAD_DIRECTION':1},
        'DIMENSION':{'UNLOADED_RADIUS':data['R0'],'WIDTH':.175,'RIM_RADIUS':.1651,
                     'RIM_WIDTH':.127,'ASPECT_RATIO':.70},
        'OPERATING_CONDITIONS':{'INFLPRES':220000,'NOMPRES':220000},
        'INERTIA':{'MASS':9.3,'IXX':.4,'IYY':.4,'BELT_MASS':data['MBELT'],'GRAVITY':-9.81},
        'VERTICAL':{'FNOMIN':data['FZ0'],'VERTICAL_STIFFNESS':220000,'VERTICAL_DAMPING':0,
                    'BREFF':0,'DREFF':0,'FREFF':0,'Q_RE0':1,'Q_V1':0,'Q_FZ2':0},
        'SCALING_COEFFICIENTS':{},'LONGITUDINAL_COEFFICIENTS':{},'LATERAL_COEFFICIENTS':{},
        'ALIGNING_COEFFICIENTS':{},'OVERTURNING_COEFFICIENTS':{},'ROLLING_COEFFICIENTS':{}}
    special={'R0','FZ0','LONGVL','MBELT','MU_REF_X','MU_REF_Y'}
    for key,value in data.items():
        if key in special:continue
        mapped='LFZO' if key=='LFZ0' else key
        if key.startswith('L'):group='SCALING_COEFFICIENTS'
        elif key.startswith('QSX'):group='OVERTURNING_COEFFICIENTS'
        elif key.startswith('QSY'):group='ROLLING_COEFFICIENTS'
        elif key.startswith(('Q','SSZ')):group='ALIGNING_COEFFICIENTS'
        elif key.startswith(('P','R')) and 'X' in key:group='LONGITUDINAL_COEFFICIENTS'
        elif key.startswith(('P','R')) and 'Y' in key:group='LATERAL_COEFFICIENTS'
        else:raise ValueError('unmapped MF52 coefficient '+key)
        sections[group][mapped]=value
    lines=['! Research comparison, copied MF52 coefficients; NOT a calibrated tire.',
           "[UNITS]","LENGTH = 'meter'","FORCE = 'newton'","ANGLE = 'radians'","MASS = 'kg'","TIME = 'second'"]
    for section,values in sections.items():
        lines.append('['+section+']')
        lines.extend(k+' = '+(v if isinstance(v,str) else format(v,'.10g')) for k,v in values.items())
    return '\n'.join(lines)+'\n',dict(modification='MF_TYRE_52_EXTERNAL_COMPARISON',adopted=False,
        is_same_vehicle_identity=False,source_coefficient_count=len(data),
        coefficient_mapping='P52_ prefix removed; LFZ0 -> LFZO; special geometry/load fields mapped',
        assumptions=['175/70R13 rim width127mm and mass9.3kg are unverified tire-file metadata',
                     'External rigid tire inertia overridden to original IT=.4; native IW=.9 retained',
                     'Linear vertical stiffness220000N/m; no added vertical damping; constant nominal RRE',
                     'VXLOW=.2m/s; dynamics switch3114 (smooth road, linear transient, symmetric combined)'],
        omitted_internal_scalars={k:data[k] for k in ('MU_REF_X','MU_REF_Y')})


def install_mf52_comparison(source,tir_path,program_dir):
    """Native model8 replaces native model7 including its shear installation."""
    if not str(tir_path).isascii():raise ValueError('external tire requires ASCII path')
    program=str(program_dir).replace('/','\\')
    install=('OPT_TIRE_INERTIA_EXTERNAL 1\n'
             'EXTERNAL_TIRE_MODULE_DLL '+program+'\\Resources\\Extensions\\External_Tire_API\\win64\\DelftTyreWrapper.dll\n'
             'INSTALL_EXTERNAL_TIRE_API\nDELFT_TYRE_DATA '+str(tir_path).replace('/','\\')+'\n'
             'DELFT_TYRE_DLL '+program+'\\Resources\\Extensions\\TNO_DelftTyre\\win64\\tno_delfttyre.dll\n'
             'DELFT_TYRE_SW 3114\nDELFT_TYRE_INIT')
    source,count=re.subn(r'INSTALL_TIRE_PAC52[^\n]*\nBEGIN_PACEJKA_52_DATA\n.*?END_PACEJKA_52_DATA',lambda _:install,source,flags=re.S)
    if count!=4:raise ValueError('expected four MF52 installations')
    source,count=re.subn(r'(?m)^OPT_TIRE_MODEL 7$','OPT_TIRE_MODEL 8',source)
    if count!=4:raise ValueError('expected four built-in MF52 tires')
    return source
