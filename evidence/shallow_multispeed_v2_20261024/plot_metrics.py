"""Plot verified frozen evidence; no solver calls or source data mutations."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
analysis=json.loads((root/'verified_analysis.json').read_text(encoding='utf-8'))
fig,axes=plt.subplots(2,2,figsize=(11,7.5))
for case,label,color in [('right_track','Right track','#21679b'),('wide','Wide pit','#de8727')]:
    rows=sorted([r for r in analysis['rows'] if r['case']==case and r['kind'] in ('validation','intermediate') and r['depth_m']==.04],key=lambda r:r['crossing_kph'])
    speed=[r['crossing_kph'] for r in rows]
    for ax,key in zip(axes.flat,['crossing_mean_kph','loaded_slip_proxy_peak_m_s','az_rms_m_s2','pitch_peak_deg']):
        values=[r['metrics'][key] for r in rows]
        ax.plot(speed,values,'o-',color=color,label=label,ms=5)
        for r,x,y in zip(rows,speed,values):
            if r['status']!='SIMULATION_PASS':ax.plot(x,y,'x',color='#b32222',ms=10,mew=2)
        ax.grid(alpha=.25);ax.set_xlabel('Requested crossing speed (km/h)')
axes[0,0].plot([1,5],[1,5],'--',color='gray',lw=1,label='Requested = measured')
axes[0,0].set_ylabel('Measured mean speed (km/h)');axes[0,0].legend(fontsize=9)
axes[0,1].axhline(.35,color='#b32222',ls=':',label='0.35 m/s research limit')
axes[0,1].set_ylabel('Loaded wheel-speed error peak (m/s)');axes[0,1].legend(fontsize=9)
axes[1,0].set_ylabel('Body vertical acceleration RMS (m/s²)')
axes[1,1].set_ylabel('Pitch peak (deg)')
fig.suptitle('Expert two: one V2 configuration, 4 cm pit, MF52 research plant\nRed crosses mark cases failing any joint screen; no full safety acceptance',fontsize=12)
fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(root/'multispeed_metrics.png',dpi=160);plt.close(fig)
