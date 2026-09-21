"""Render measured RoboDK trace diagnostics with explicit finite-difference units."""
import csv
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    root=Path(sys.argv[1])
    arms=['A_stop','B_smooth_fixed','C_smooth_variable']
    fig,axes=plt.subplots(3,3,figsize=(12,8),constrained_layout=True)
    plt.rcParams['svg.fonttype']='none'
    summary={}
    for row,arm in enumerate(arms):
        raw=json.loads((root/(arm+'_trajectory.json')).read_text(encoding='utf-8'))
        r=np.asarray(raw['rows'],dtype=float)
        dt=r[10]; t=np.cumsum(dt); keep=(r[9]>1)&(dt>1e-9)
        distance=np.r_[0.,np.linalg.norm(np.diff(r[11:14],axis=1),axis=0)]
        speed=np.divide(distance,dt,out=np.zeros_like(dt),where=dt>1e-9)
        qd=np.diff(r[:6],axis=1)/np.maximum(dt[1:],1e-9)
        qdd=np.diff(qd,axis=1)/np.maximum((dt[2:]+dt[1:-1])/2,1e-9)
        series=[(t[keep],speed[keep]),(t[1:][keep[1:]],np.max(abs(qd),axis=0)[keep[1:]]),
                (t[2:][keep[2:]],np.max(abs(qdd),axis=0)[keep[2:]])]
        for col,(x,y) in enumerate(series):
            ax=axes[row,col]; ax.plot(x,y,color='black',linewidth=.65)
            ax.set_xlabel('Trace time (s)')
            ax.set_ylabel(['TCP interval speed (mm/s)','Peak joint FD speed (deg/s)','Peak joint FD acceleration (deg/s²)'][col])
            ax.text(.02,.95,arm[0],transform=ax.transAxes,va='top',fontweight='bold')
            ax.spines[['top','right']].set_visible(False)
            if col:
                ax.axhline([0,60,180][col],color='gray',linestyle='--',linewidth=.8)
            if col==2:
                ax.set_yscale('symlog',linthresh=10)
                ax.set_ylim(bottom=0)
        with (root/(arm+'_curves.csv')).open('w',newline='',encoding='utf-8') as stream:
            writer=csv.writer(stream)
            writer.writerow(['time_s','move_id','dt_s','tcp_interval_speed_mm_s']+
                            ['joint_'+str(j+1)+'_deg' for j in range(6)])
            writer.writerows(zip(t,r[9],dt,speed,*r[:6]))
        summary[arm]={'trace_scan_time_s':float(dt[keep].sum()),
                      'api_status':raw['status'],'maximum_error_code':float(r[6].max())}
    fig.savefig(root/'trace_diagnostics.png',dpi=180)
    fig.savefig(root/'trace_diagnostics.svg')
    plt.close(fig)
    (root/'plot_notes.json').write_text(json.dumps({'summary':summary,
        'scope':'Diagnostic failed first comparison; FD denotes finite differences from joint positions and trace dt.',
        'limits':'Dashed lines are simulation settings, not verified UR10 hardware limits.',
        'stop_detection':'Interval-average speed does not prove nonzero instantaneous velocity at a target.',
        'timing':'Trajectory dt sum, not wall-clock playback or Update cycle estimate.'},indent=2),encoding='utf-8')


if __name__=='__main__':
    main()
