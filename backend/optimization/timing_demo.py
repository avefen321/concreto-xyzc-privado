"""Analytic demonstration with ASSUMED limits, never machine calibration."""
import argparse
import math
from pathlib import Path
from ..core.primitives import ArcPrimitive
from ..core.motion_timing import estimate_time
from .benchmark import write,csv_file


def run(directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False)
    arc=ArcPrimitive(id='P1',source_id=1,layer_id=1,stroke_id=1,type='ARC',
        start_xyz=(50.,0.,20.),end_xyz=(50.,0.,20.),start_c=0.,end_c=360.,feed=2400.,
        process_state='DEPOSIT',source_line=1,sample_id=1,center_xyz=(0.,0.,20.),
        radius=50.,sweep_rad=2*math.pi,direction='CCW',start_angle_rad=0.)
    limits={axis:{'velocity':100.,'acceleration':200.} for axis in 'XYZ'}
    limits['C']={'velocity':90.,'acceleration':180.}
    profile=estimate_time([arc],limits,policy='CONTINUOUS_TANGENT')
    profile['limits_origin']='ASSUMED_FOR_ANALYTIC_TEST_NOT_MEASURED_ON_MACHINE'
    write(directory/'kinematic_profile.json',profile)
    segment=profile['segments'][0];a=segment['acceleration_cap'];peak=segment['peak_velocity']
    up,cruise=segment['accel_seconds'],segment['cruise_seconds']; rows=[]
    for index in range(501):
        t=profile['t_sim_s']*index/500
        if t<=up: s=.5*a*t*t;v=a*t;acc=a
        elif t<=up+cruise: s=.5*a*up*up+peak*(t-up);v=peak;acc=0.
        else:
            remaining=profile['t_sim_s']-t;s=arc.length_mm-.5*a*remaining*remaining;v=a*remaining;acc=-a
        pose=arc.pose(s/arc.length_mm)
        rows.append({'t_s':t,'s_mm':s,'v_mm_s':v,'a_mm_s2':acc,'C_deg':pose[3],
                     'omega_C_deg_s':v*360/arc.length_mm})
    csv_file(directory/'kinematic_series.csv',rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(5,1,figsize=(9,10),sharex=True)
    for axis,key in zip(axes,('s_mm','v_mm_s','a_mm_s2','C_deg','omega_C_deg_s')):
        axis.plot([r['t_s'] for r in rows],[r[key] for r in rows]);axis.set_ylabel(key);axis.grid(alpha=.2)
    axes[0].set_title('Modelo matemático · límites supuestos · sin medición CNC')
    axes[-1].set_xlabel('Tiempo estimado (s)');fig.tight_layout();fig.savefig(directory/'kinematic_comparison.png',dpi=150);plt.close(fig)
    return profile


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);args=p.parse_args();run(args.output)
