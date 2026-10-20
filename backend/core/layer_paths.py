"""Continuidad entre capas para trazos abiertos repetidos.

La fuente no se reescribe. Solo se reorganiza el plan canónico cuando dos
capas consecutivas contienen el mismo trazo abierto y la inversión puede
certificarse sin cruzar eventos de proceso, feeds ni geometría protegida.
"""
from collections import Counter
import math

from .gcode import arc_geometry
from .path import build, xy_length


def _same_xy(a, b, tolerance):
    return len(a) == len(b) and all(math.dist(p.xyz[:2], q.xyz[:2]) <= tolerance
                                    for p, q in zip(a, b))


def _closed(stroke, tolerance):
    return len(stroke.points) > 3 and xy_length(stroke.points[0].xyz,
                                                stroke.points[-1].xyz) <= tolerance


def _transition_is_safe(motions, start, end):
    for motion in motions[start:end]:
        if (motion.deposition and motion.kind != 'Z_ONLY') or motion.kind not in ('RAPID', 'Z_ONLY'):
            return False
        if motion.block.value('C') is not None or motion.block.value('E') is not None:
            return False
        if motion.arc or motion.cordon_role or motion.layer_closure_role:
            return False
    return True


def _hold_transition_xy(motions, start, end, xy):
    current_z=motions[start-1].end[2] if start else motions[0].start[2]
    for motion in motions[start:end]:
        motion.start=(xy[0],xy[1],current_z)
        current_z=motion.end[2]
        motion.end=(xy[0],xy[1],current_z)


def _reverse_stroke_motions(stroke, motions):
    indices=[p.motion_index for p in stroke.points[1:]]
    if len(set(indices)) != len(indices) or indices != list(range(indices[0],indices[-1]+1)):
        return False
    descriptors=[]
    for index in indices:
        motion=motions[index]
        center=None
        if motion.kind in ('ARC_CW','ARC_CCW'):
            cx,cy,_,_,_=arc_geometry(motion)
            center=(cx,cy,motion.arc.get('turns',1))
        descriptors.append((motion.kind,motion.feed,center))
    points=[p.xyz for p in stroke.points]
    for rank,index in enumerate(indices):
        target=motions[index]
        source_kind,source_feed,center=descriptors[-1-rank]
        target.start=points[-1-rank]
        target.end=points[-2-rank]
        target.feed=source_feed
        if center is None:
            target.kind='LINEAR'; target.original_kind='LINEAR'; target.arc={}
        else:
            cx,cy,turns=center
            target.kind='ARC_CCW' if source_kind=='ARC_CW' else 'ARC_CW'
            target.original_kind=target.kind
            target.arc={'I':cx-target.start[0],'J':cy-target.start[1],
                        'plane':17,'center_mode':'G91.1'}
            if turns != 1: target.arc['turns']=turns
    return True


def alternate_open_layer_paths(strokes, layers, motions, config):
    report=[]
    if not config.layer_transition.alternate_open_paths:
        return strokes,layers,report,[]
    tolerance=max(config.layer_completion.match_tolerance_mm,
                  config.geometry.zero_length_tolerance_mm)
    counts=Counter(s.layer_id for s in strokes if s.deposition)
    candidates=[s for s in strokes if s.deposition and s.layer_id is not None and counts[s.layer_id]==1]
    previous=None
    previous_end=None
    previous_last_index=None
    changed=False
    warnings=[]
    for stroke in candidates:
        row={'stroke_id':stroke.id,'layer_id':stroke.layer_id,'status':'NOT APPLICABLE',
             'reversed':False,'transition_xy_held':False,'reason':'Primer trazo abierto de referencia'}
        planar=max(p.xyz[2] for p in stroke.points)-min(p.xyz[2] for p in stroke.points) <= config.layers.z_change_tolerance_mm
        motion_indices=[p.motion_index for p in stroke.points[1:]]
        uniform_feed=len({motions[i].feed for i in motion_indices}) <= 1
        protected=any(motions[i].e_start is not None or motions[i].e_end is not None for i in motion_indices)
        if _closed(stroke,tolerance) or not planar or not uniform_feed or protected:
            row['reason']='Loop cerrado, Z variable, feed variable o eje E protegido'
            report.append(row); previous=None; previous_end=None; previous_last_index=None; continue
        if previous is not None:
            same_shape=_same_xy(previous.points,stroke.points,tolerance) or _same_xy(previous.points,list(reversed(stroke.points)),tolerance)
            first,last=stroke.points[0].xyz,stroke.points[-1].xyz
            transition_start=previous_last_index+1
            transition_end=motion_indices[0]
            safe_interval=_transition_is_safe(motions,transition_start,transition_end)
            if same_shape and safe_interval and math.dist(previous_end[:2],first[:2]) <= tolerance:
                _hold_transition_xy(motions,transition_start,transition_end,previous_end[:2])
                row.update(status='PASS',transition_xy_held=True,
                           reason='La fuente ya alternaba el sentido; subida Z con XY y C constantes')
                changed=True
            elif same_shape and safe_interval and math.dist(previous_end[:2],last[:2]) <= tolerance:
                if _reverse_stroke_motions(stroke,motions):
                    _hold_transition_xy(motions,transition_start,transition_end,previous_end[:2])
                    row.update(status='PASS',reversed=True,transition_xy_held=True,
                               reason='Trazo abierto invertido para imprimir ida y vuelta sin retorno lateral')
                    changed=True
                    first,last=last,first
                else:
                    row.update(status='WARNING',reason='No se pudo invertir con trazabilidad unívoca')
                    warnings.append(f'Trazo {stroke.id}: inversión abierta omitida por trazabilidad')
            else:
                row['reason']='Geometría distinta o transición con evento protegido'
            previous_end=last
        else:
            previous_end=stroke.points[-1].xyz
        previous=stroke
        previous_last_index=motion_indices[-1]
        report.append(row)
    if not changed:
        return strokes,layers,report,warnings
    rebuilt,new_layers,rebuild_warnings=build(motions,config)
    return rebuilt,new_layers,report,warnings+rebuild_warnings
