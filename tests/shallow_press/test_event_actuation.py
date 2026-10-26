import numpy as np
from test_actuation import feedback


def test_airborne_sync_is_small_local_and_cannot_bypass_stale_unloading():
    from ddevsim.strategies.shallow_press.actuation import CrawlActuation
    from ddevsim.strategies.shallow_press.config import ShallowPressConfig
    c=ShallowPressConfig(policy='event_crawl');a=CrawlActuation(c);d=feedback()
    for w in ('L1','R1','L2','R2'):d['Fz_'+w]=0.;d['CmpT_'+w+'i']=0.;d['AVy_'+w]=1.3/.263
    context=dict(phases=['BYPASS','ENTRY','BYPASS','CLIMB'],airborne_sync_gain=50.,airborne_sync_limit_nm=30.)
    old=np.zeros(9)
    for t in np.arange(0,.1,.0005):
        u=a.step(float(t),float(t),d,np.zeros(9),None,{'friction':.7},.0005,event_context=context)
        assert max(abs(u[:4]-old[:4]))<=.50000001;old=u
    assert u[0]==u[2]==0 and -30<=u[1]<-20 and -30<=u[3]<-20
    for t in np.arange(.1,.3,.0005):u=a.step(float(t),0.,d,np.zeros(9),None,{'friction':.7},.0005,event_context=context)
    assert np.allclose(u,0.)
