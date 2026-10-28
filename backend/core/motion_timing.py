"""Conservative time parametrization of the FINAL XYZC primitives, not LinuxCNC.

q'(s), q''(s) bounds share acceleration capacity between curvature and
tangential acceleration. No jerk, extrusion, controller look-ahead or material.
"""
import math

AXES=('X','Y','Z','C')


def trapezoid(length,velocity,acceleration,entry=0.,exit=0.):
    if length<=1e-12: return {'seconds':0.,'peak_velocity':0.,'accel_seconds':0.,'cruise_seconds':0.,'decel_seconds':0.}
    peak=min(velocity,math.sqrt(acceleration*length+(entry*entry+exit*exit)/2))
    up=max(0.,(peak-entry)/acceleration); down=max(0.,(peak-exit)/acceleration)
    covered=(peak*peak-entry*entry)/(2*acceleration)+(peak*peak-exit*exit)/(2*acceleration)
    cruise=max(0.,length-covered)/peak
    return {'seconds':up+cruise+down,'peak_velocity':peak,'accel_seconds':up,'cruise_seconds':cruise,'decel_seconds':down}


def derivative_at(p,t):
    length=p.length_mm
    if length<1e-12: return (0.,0.,0.,math.copysign(1,p.end_c-p.start_c) if p.end_c!=p.start_c else 0.)
    xyz=[(b-a)/length for a,b in zip(p.start_xyz,p.end_xyz)]
    if p.type=='ARC':
        angle=p.start_angle_rad+p.sweep_rad*t
        xyz[:2]=[-p.radius*math.sin(angle)*p.sweep_rad/length,
                  p.radius*math.cos(angle)*p.sweep_rad/length]
    return (*xyz,(p.end_c-p.start_c)/length)


def estimate_time(primitives,limits=None,*,policy=None,gcode=''):
    result={'status':'NOT_CONFIGURED','t_sim_s':None,'t_real_s':None,
            'model':'BOUNDED_ACCELERATION_EXACT_PATH_V1','policy':policy,'segments':[],
            'assumptions':['No jerk limit','No extrusion/rheology','Not a LinuxCNC planner replica'],
            'limits':limits,'reason':None}
    if not limits or policy not in ('EXACT_STOP','CONTINUOUS_TANGENT'):
        result['reason']='Defina límites XYZC y política de transición'; return result
    for axis in AXES:
        data=limits.get(axis,{})
        if any(type(data.get(k)) not in (float,int) or not math.isfinite(data[k]) or data[k]<=0 for k in ('velocity','acceleration')):
            result['reason']='Límite ausente o inválido: '+axis; return result
    if gcode:
        from .gcode import parse
        if any(k=='G' and v==4 for b in parse(gcode) for k,v in b.words):
            result.update(status='NOT_EVALUATED',reason='Pausas G4 no modeladas'); return result
    segments=[]
    for p in primitives:
        length=p.length_mm if p.length_mm>1e-12 else abs(p.end_c-p.start_c)
        d0=derivative_at(p,0); d1=derivative_at(p,1)
        first=[max(abs(a),abs(b)) for a,b in zip(d0,d1)]; second=[0.]*4
        if p.type=='ARC' and p.length_mm>1e-12:
            # Global sin/cos <= 1 bounds remain conservative for any sweep.
            first[0]=first[1]=abs(p.radius*p.sweep_rad/p.length_mm)
            second[0]=second[1]=p.radius*(p.sweep_rad/p.length_mm)**2
        vmax=math.inf; amax=math.inf
        for i,axis in enumerate(AXES):
            v,a=limits[axis]['velocity'],limits[axis]['acceleration']
            if first[i]>1e-12:
                vmax=min(vmax,v/first[i]); amax=min(amax,a/(2 if second[i]>0 else 1)/first[i])
            if second[i]>1e-12: vmax=min(vmax,math.sqrt(a/2/second[i]))
        if length>1e-12 and p.type!='TRAVEL':
            if not p.feed or p.feed<=0:
                result.update(status='NOT_EVALUATED',reason='Movimiento sin F: '+p.id); return result
            vmax=min(vmax,p.feed/60.)
        if length<=1e-12: vmax=amax=1.
        segments.append({'primitive_id':p.id,'length':length,'coordinate':'mm' if p.length_mm>1e-12 else 'degree',
                         'velocity_cap':vmax,'acceleration_cap':amax,'entry_derivative':d0,'exit_derivative':d1})
    speeds=[0.]*(len(segments)+1)
    if policy=='CONTINUOUS_TANGENT':
        for i in range(1,len(segments)):
            left,right=segments[i-1],segments[i]
            if (left['coordinate']==right['coordinate'] and
                    max(abs(a-b) for a,b in zip(left['exit_derivative'],right['entry_derivative']))<1e-8):
                speeds[i]=min(left['velocity_cap'],right['velocity_cap'])
    for i,p in enumerate(segments):
        speeds[i+1]=min(speeds[i+1],math.sqrt(speeds[i]**2+2*p['acceleration_cap']*p['length']))
    for i in range(len(segments)-1,-1,-1):
        p=segments[i]; speeds[i]=min(speeds[i],math.sqrt(speeds[i+1]**2+2*p['acceleration_cap']*p['length']))
    total=0.
    for i,p in enumerate(segments):
        profile=trapezoid(p['length'],p['velocity_cap'],p['acceleration_cap'],speeds[i],speeds[i+1])
        p.update(profile,entry_velocity=speeds[i],exit_velocity=speeds[i+1],start_time_s=total)
        total+=profile['seconds']; p['end_time_s']=total
    result.update(status='ESTIMATED',t_sim_s=total,segments=segments)
    return result
