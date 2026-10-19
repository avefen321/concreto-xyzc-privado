"""Physical block metrics from the final plan; no machine-time prediction."""
import math
import statistics


def density(samples,input_points,config):
    previous={}; lengths=[]; deltas=[]; counts={'straight_G1_count':0,'curve_G1_count':0,'corner_G1_count':0}
    corner_time=0.
    for s in samples:
        if not s.deposition_active: continue
        a=previous.get(s.stroke_id)
        if a and s.emit and s.motion_type!='C_ONLY':
            length=math.dist((a.X,a.Y,a.Z),(s.X,s.Y,s.Z))
            s.segment_length_mm=length
            lengths.append(length);deltas.append(abs(s.C_deg-a.C_deg))
            key='corner_G1_count' if s.generated_by_fillet else 'straight_G1_count' if s.primitive_type in ('LINE','STRAIGHT') and abs(s.C_deg-a.C_deg)<1e-8 else 'curve_G1_count'
            counts[key]+=1
            if s.generated_by_fillet and s.duration_s: corner_time+=s.duration_s
        previous[s.stroke_id]=s
    return {'input_points':input_points,'output_G1_count':sum(s.emit and s.motion_type not in ('RAPID','PREP_RAPID') for s in samples),
            **counts,'min_block_length_mm':min(lengths,default=0.),
            'median_block_length_mm':statistics.median(lengths) if lengths else 0.,
            'max_block_length_mm':max(lengths,default=0.),
            'microblocks_lt_0_5mm':sum(x<.5 for x in lengths),
            'microblocks_lt_1mm':sum(x<1 for x in lengths),'microblocks_lt_2mm':sum(x<2 for x in lengths),
            'preferred_min_block_length_mm':config.sampling.preferred_min_block_length_mm,
            'microblock_count':sum(x<config.sampling.preferred_min_block_length_mm for x in lengths),
            'max_delta_C':max(deltas,default=0.),'median_delta_C':statistics.median(deltas) if deltas else 0.,
            'corner_ideal_time_s':corner_time,'time_model':'Cartesian chord length / source or planned feed; excludes controller acceleration/blending'}


def primitive_density(primitives,input_points,config):
    printed=[p for p in primitives if p.process_state=='DEPOSIT' and p.type not in ('C_ONLY','TRAVEL')]
    linear=[p for p in printed if p.type!='ARC']; lengths=[p.length_mm for p in printed]
    deltas=[abs(p.end_c-p.start_c) for p in printed]
    return {'input_points':input_points,'output_G1_count':sum(p.type not in ('ARC','TRAVEL') for p in primitives),
            'output_G2_count':sum(p.type=='ARC' and p.direction=='CW' for p in primitives),
            'output_G3_count':sum(p.type=='ARC' and p.direction=='CCW' for p in primitives),
            'straight_G1_count':sum(abs(p.end_c-p.start_c)<1e-8 for p in linear),
            'curve_G1_count':sum(abs(p.end_c-p.start_c)>=1e-8 for p in linear),'corner_G1_count':0,
            'min_block_length_mm':min(lengths,default=0.),'median_block_length_mm':statistics.median(lengths) if lengths else 0.,
            'max_block_length_mm':max(lengths,default=0.),
            'microblocks_lt_0_5mm':sum(x<.5 for x in lengths),'microblocks_lt_1mm':sum(x<1 for x in lengths),
            'microblocks_lt_2mm':sum(x<2 for x in lengths),
            'microblock_count':sum(p.length_mm<config.sampling.preferred_min_block_length_mm for p in linear),
            'preferred_min_block_length_mm':config.sampling.preferred_min_block_length_mm,
            'max_delta_C':max(deltas,default=0.),'median_delta_C':statistics.median(deltas) if deltas else 0.,
            'corner_ideal_time_s':sum(p.duration_s or 0 for p in printed if p.type=='ARC' and p.arc_source=='CORNER_FILLET'),
            'time_model':'Native 3D arc/line length / feed; excludes controller acceleration/blending'}
