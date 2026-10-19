"""Feed constraints in physical time; millimetres and degrees are never added."""
import math


def plan_field(field, config):
    rows=[]
    for i,p in enumerate(field.stroke.points[1:]):
        gradient=field.max_dc_ds(i)
        limit=config.kinematics.max_c_velocity_deg_s
        maximum=60*limit/gradient if limit and gradient>config.precision.angle_epsilon_deg else None
        recommendation=min(p.feed,maximum) if p.feed and maximum else p.feed
        rows.append({'stroke_id':field.stroke.id,'primitive_id':f'{field.stroke.id}:{i}',
                     'source_line':p.line,'feed_source':p.feed,'max_dc_ds_deg_mm':gradient,
                     'max_feed_by_c_mm_min':maximum,'recommended_feed_mm_min':recommendation,
                     'reason':'Límite de velocidad C' if maximum and p.feed and maximum<p.feed else
                              'Límite C desconocido; conservar F' if not limit else 'F compatible con velocidad C'})
    # Immediate look-ahead remains inside this field/stroke. Avoid abrupt changes of F
    # by extending the stricter constraint to the adjoining primitive.
    planned=[]
    for i,row in enumerate(rows):
        feed=row['feed_source']
        if config.process.feed_policy=='ADAPTIVE_EXPERIMENTAL' and feed is not None and (
                config.kinematics.max_c_velocity_deg_s or config.kinematics.max_c_acceleration_deg_s2):
            feed*=config.process.experimental_feed_factor
            caps=[r['max_feed_by_c_mm_min'] for r in rows[max(0,i-1):i+2] if r['max_feed_by_c_mm_min'] is not None]
            if caps: feed=min(feed,*caps)
        planned.append(feed)
        row['feed_planned']=feed
    return planned,rows


def enforce_acceleration(samples, config, analyze):
    """Final discrete-time gate after sampling, before any serialization.

    A common scale per stroke preserves the planned relative speed profile. With
    a scale k, omega scales as k and alpha as k². No cross-stroke look-ahead.
    """
    report,warnings=analyze(samples,config)
    if config.process.feed_policy!='ADAPTIVE_EXPERIMENTAL': return report,warnings
    grouped={}
    for s in samples:
        if s.deposition_active: grouped.setdefault(s.stroke_id,[]).append(s)
    for members in grouped.values():
        omega=max((abs(s.omega_C_deg_s) for s in members if s.omega_C_deg_s is not None),default=0.)
        alpha=max((abs(s.alpha_C_deg_s2) for s in members if s.alpha_C_deg_s2 is not None),default=0.)
        factors=[1.]
        if config.kinematics.max_c_velocity_deg_s and omega: factors.append(config.kinematics.max_c_velocity_deg_s/omega)
        if config.kinematics.max_c_acceleration_deg_s2 and alpha: factors.append(math.sqrt(config.kinematics.max_c_acceleration_deg_s2/alpha))
        scale=min(factors)
        if scale<1:
            for s in members:
                if s.feed_mm_min is not None: s.feed_mm_min*=scale*(1-1e-9)
    return analyze(samples,config)
