"""Validate rounded native-arc geometry before handing it to LinuxCNC."""
import math
from .gcode import Motion, Block, arc_geometry


def fixed(value,decimals):
    value=round(value,decimals)
    return f'{0. if value==0 else value:.{decimals}f}'


def arc_words(p,config):
    precision=config.serialization
    start=tuple(float(fixed(x,precision.xyz_decimals)) for x in p.start_xyz)
    end=tuple(float(fixed(x,precision.xyz_decimals)) for x in p.end_xyz)
    if (p.plane!='G17' or not all(math.isfinite(x) for x in (*p.center_xyz,p.radius,p.sweep_rad)) or p.radius<=0):
        raise ValueError(f'{p.id}: geometría de arco inválida')
    original_r0=math.dist(p.start_xyz[:2],p.center_xyz[:2]); original_r1=math.dist(p.end_xyz[:2],p.center_xyz[:2])
    if abs(original_r0-original_r1)>.01: raise ValueError(f'{p.id}: radios internos inconsistentes')
    requested=config.arc_output.format
    if requested=='AUTO':
        policy=config.native_arcs.corner_fillets if p.arc_source=='CORNER_FILLET' else config.native_arcs.source_arcs if p.arc_source=='SOURCE_G2G3' else config.native_arcs.fitted_free_curves
        requested=policy.format
    radius_format=requested=='RADIUS_R' or requested=='AUTO' and p.arc_source=='CORNER_FILLET'
    safe_r=abs(math.degrees(p.sweep_rad))<config.native_arcs.radius_format_max_sweep_deg and math.dist(start[:2],end[:2])>1e-9
    def resolve(arc):
        m=Motion(p.source_line,'ARC_CCW' if p.direction=='CCW' else 'ARC_CW',start,end,p.feed,True,Block(p.source_line,'',[],[]),arc=arc)
        return arc_geometry(m)
    geometry_budget=math.sqrt(2)*10**(-precision.xyz_decimals)+2*10**(-precision.ijk_decimals)+1e-8
    if radius_format and safe_r:
        radius=float(fixed(p.radius,precision.radius_decimals))
        try:
            cx,cy,r,a,sweep=resolve({'R':radius})
            if math.dist((cx,cy),p.center_xyz[:2])<=geometry_budget and abs(sweep-p.sweep_rad)*p.radius<=geometry_budget:
                return 'R'+fixed(radius,precision.radius_decimals)
        except ValueError: pass
    if radius_format and not config.native_arcs.auto_fallback_to_ijk:
        raise ValueError(f'{p.id}: R inseguro y fallback I/J deshabilitado')
    i=float(fixed(p.center_xyz[0]-start[0],precision.ijk_decimals))
    j=float(fixed(p.center_xyz[1]-start[1],precision.ijk_decimals))
    center=(start[0]+i,start[1]+j)
    r0=math.dist(start[:2],center); r1=math.dist(end[:2],center)
    # This gate is deliberately stricter than LinuxCNC's radius mismatch gate.
    if abs(r0-r1)>.005+1e-9:
        raise ValueError(f'{p.id}: radius to end differs from start ({abs(r0-r1):.6f} mm); aumente decimales XYZ/IJ')
    cx,cy,r,a,sweep=resolve({'I':i,'J':j})
    if abs(sweep-p.sweep_rad)*p.radius>geometry_budget:
        raise ValueError(f'{p.id}: redondeo altera el barrido; aumente decimales XYZ/IJ')
    return 'I'+fixed(i,precision.ijk_decimals)+' J'+fixed(j,precision.ijk_decimals)
