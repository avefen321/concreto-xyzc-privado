"""Semantic layer schedule, separate from XY geometry and orientation."""
from dataclasses import replace
import math


def separate_layer_entries(motions, config):
    """Recognize a step into a Z plateau; retain genuine continuous 3D ramps.

    A simultaneous XY/Z entry followed by a planar deposition move is expanded
    into a vertical transition and its original XY endpoint at the new layer.
    This is an explicit topology conversion, never a textual Z substitution.
    """
    if not config.z_conversion.enabled: return motions, []
    result=[]; warnings=[]
    tol=config.layers.z_change_tolerance_mm
    for i,m in enumerate(motions):
        following=motions[i+1] if i+1<len(motions) else None
        plateau=(following and following.deposition and following.kind=='LINEAR' and
                 abs(following.start[2]-m.end[2])<=tol and
                 abs(following.end[2]-m.end[2])<=tol)
        if (m.deposition and m.kind=='LINEAR' and abs(m.end[2]-m.start[2])>tol and
                math.dist(m.start[:2],m.end[:2])>tol and plateau):
            if m.e_end is not None:
                raise ValueError(f'Línea {m.line}: entrada de capa diagonal con E requiere política explícita; desactive normalización Z para conservar la rampa')
            raised=(*m.start[:2],m.end[2])
            result.append(replace(m,end=raised,kind='Z_ONLY',original_kind='LINEAR',deposition=False))
            result.append(replace(m,start=raised))
            warnings.append(f'Línea {m.line}: LAYER_ENTRY_SPLIT: entrada XY/Z a meseta separada en Z-only y XY; extremos XY conservados')
        else: result.append(m)
    return result,warnings


def normalize_layers(strokes, layers, motions, config):
    cfg=config.z_conversion
    sources={layer.id:layer.nominal_z for layer in layers}
    targets={layer.id:(cfg.first_layer_z_mm+cfg.subsequent_layer_height_mm*k
                       if cfg.enabled and cfg.apply_to_deposition_layers else layer.nominal_z)
             for k,layer in enumerate(layers)}
    rows=[]
    tol=cfg.z_match_tolerance_mm
    dep=[(i,m.layer_id) for i,m in enumerate(motions) if m.layer_id is not None]
    next_layer={}; lid=None
    for i in range(len(motions)-1,-1,-1):
        if motions[i].layer_id is not None: lid=motions[i].layer_id
        next_layer[i]=lid
    previous_layer=None
    for i,m in enumerate(motions):
        source_start,source_end=m.start,m.end
        category='TRAVEL_Z'; reason='PRESERVED_TRAVEL'; layer=m.layer_id
        explicit_safe='MACHINE_SAFE_Z' in m.block.raw.upper()
        delta=0.
        if m.layer_id is not None:
            previous_layer=m.layer_id
            category='DEPOSITION_Z'; reason='NORMALIZED_LAYER_SCHEDULE'
            delta=targets[layer]-sources[layer]
        elif explicit_safe and cfg.preserve_explicit_machine_safe_travel_z:
            category='MACHINE_SAFE_Z'; reason='EXPLICIT_MACHINE_SAFE_Z'
        elif cfg.apply_to_layer_transitions:
            candidate=next_layer[i]
            if candidate is not None and abs(source_end[2]-sources[candidate])<=tol:
                layer=candidate; delta=targets[layer]-sources[layer]
                category='LAYER_TRANSITION_Z' if m.kind=='Z_ONLY' else 'TRAVEL_Z'
                reason='LAYER_ENTRY_Z'
            elif previous_layer is not None and ('LAYER_RELATIVE_Z' in m.block.raw.upper() or candidate==previous_layer):
                layer=previous_layer; delta=targets[layer]-sources[layer]
                reason='LAYER_RELATIVE_CLEARANCE'
        if cfg.enabled:
            m.end=(*m.end[:2],source_end[2]+delta)
            # Preserve the actual previous Cartesian endpoint for transition time.
            m.start=(*m.start[:2],motions[i-1].end[2] if i else source_start[2])
            if category=='DEPOSITION_Z': m.start=(*m.start[:2],source_start[2]+delta)
        rows.append({'motion_index':i,'source_line':m.line,'layer_id':layer,
                     'layer_index':layer-1 if layer else None,'z_source':source_end[2],
                     'z_source_start':source_start[2],'z_target':m.end[2],
                     'z_conversion_applied':cfg.enabled and abs(delta)>1e-12,
                     'z_conversion_reason':reason if cfg.enabled else 'DISABLED','z_role':category})
    for stroke in strokes:
        if stroke.layer_id is not None:
            delta=targets[stroke.layer_id]-sources[stroke.layer_id]
            if cfg.enabled:
                for p in stroke.points: p.xyz=(*p.xyz[:2],p.xyz[2]+delta)
        else:
            for j,p in enumerate(stroke.points):
                m=motions[p.motion_index]
                p.xyz=m.start if j==0 else m.end
    for layer in layers: layer.nominal_z=targets[layer.id]
    return {'enabled':cfg.enabled,'source_layer_z_values':list(sources.values()),
            'target_layer_z_values':list(targets.values()),'first_layer_target_mm':cfg.first_layer_z_mm,
            'subsequent_layer_height_mm':cfg.subsequent_layer_height_mm,
            'z_conversion_count':sum(r['z_conversion_applied'] for r in rows),
            'z_transition_count':sum(r['z_role']=='LAYER_TRANSITION_Z' for r in rows),
            'unexpected_z_count':0,'max_layer_z_error_mm':0.,'motions':rows}
