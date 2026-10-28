"""G1 control of the SAME frozen reference, using a conservative error budget.

Arc subdivision interpolation error <= R * delta_angle**2 / 8.  C is
interpolated with the same parameter as the parent primitive.  The parent's
independent bound and a fresh rounding budget are added, never reset to zero.
"""
from copy import deepcopy
from dataclasses import fields,replace
import math
from ..core.conversion import Result
from ..core.primitives import MotionPrimitive
from ..core.states import FinalMotionPlan,digest
from ..core.gcode import parse,interpret
from ..core.reference_snapshot import semantic_signature
from ..core.validators import NativeArcValidator,SerializationValidator
from .baseline import exported_metrics
from .evaluator import Evaluation,validate_candidate


def dense_control(snapshot,baseline):
    parent=validate_candidate(snapshot,baseline)
    if parent['status']!='PASS': return Evaluation(None,{'feasible':False,'failure_reason':'Parent reference not certified'})
    config=snapshot.config; precision=config.serialization
    xyz_round=5*math.sqrt(3)*10**(-precision.xyz_decimals)+2*10**(-precision.ijk_decimals)
    c_round=5*10**(-precision.c_decimals)
    geometry=min(config.arc_fit.geometry_tolerance_mm,config.sampling.geometry_error_mm)-parent['xy_error_mm']-xyz_round
    angle=config.arc_fit.orientation_tolerance_deg-parent['C_error_deg']-c_round
    if geometry<=0 or angle<=0: return Evaluation(None,{'feasible':False,'failure_reason':'No remaining dense-control error budget'})
    block_map={row['output_line']:row['sample_id'] for row in baseline.report['serialization_map'] if row['sample_id'] is not None}
    by_sample={p.sample_id:p for p in baseline.final_motion_plan.primitives}
    samples_by_id={s.sample_id:s for s in baseline.samples}; new_samples=[]; primitives=[]; lines=[]; mapping=[]
    max_extra=0.; identifier=digest([snapshot.hash,'G1_DENSE_REFERENCE_V1'])
    local_bounds={row.get('primitive'):row for row in parent['regions']}
    keys={f.name for f in fields(MotionPrimitive)}
    for number,text in enumerate(baseline.gcode.splitlines(),1):
        if number not in block_map:
            lines.append(text.replace(baseline.report['conversion_id'],identifier)); continue
        p=by_sample[block_map[number]]
        if p.E is not None: return Evaluation(None,{'feasible':False,'failure_reason':'E-axis splitting not supported'})
        n=max(1,math.ceil(p.length_mm/config.sampling.max_linear_step_mm)) if p.type=='LINE' else 1
        if p.type=='ARC':
            local_angle=1.-local_bounds.get(p.id,{}).get('C_bound_deg',c_round)-c_round
            if local_angle<=0: return Evaluation(None,{'feasible':False,'failure_reason':'Arc angle budget exhausted'})
            step=min(math.radians(min(1.,local_angle*.5)),math.sqrt(8*geometry/p.radius))
            n=max(1,math.ceil(abs(p.sweep_rad)/step))
            max_extra=max(max_extra,p.radius*(p.sweep_rad/n)**2/8)
        if len(primitives)+n>100000: return Evaluation(None,{'feasible':False,'failure_reason':'Dense control exceeds 100000 blocks'})
        for k in range(n):
            start=p.pose(k/n); end=p.pose((k+1)/n); sample_id=len(primitives)+1
            kwargs={key:getattr(p,key) for key in keys}
            kwargs.update(id=f'P{sample_id}',sample_id=sample_id,start_xyz=tuple(start[:3]),end_xyz=tuple(end[:3]),
                          start_c=start[3],end_c=end[3],type='LINE' if p.type=='ARC' else p.type,
                          metadata=dict(p.metadata,control_reference_primitive=p.id,control_fraction=[k/n,(k+1)/n]))
            item=MotionPrimitive(**kwargs); primitives.append(item)
            template=samples_by_id[p.sample_id]
            sample=replace(template,sample_id=sample_id,X=end[0],Y=end[1],Z=end[2],C_deg=end[3],
                           emit=True,display_only=False,primitive_id=item.id)
            new_samples.append(sample)
            fixed=lambda value,decimals:f'{0. if round(value,decimals)==0 else value:.{decimals}f}'
            command='G0' if item.type=='TRAVEL' else 'G1'
            if item.type!='C_ONLY':
                command+=' '+' '.join(axis+fixed(value,precision.xyz_decimals) for axis,value in zip('XYZ',end[:3]))
            command+=' C'+fixed(end[3],precision.c_decimals)
            if item.feed and item.type!='TRAVEL': command+=' F'+fixed(item.feed,precision.feed_decimals)
            lines.append(command); mapping.append({'output_line':len(lines),'sample_id':sample_id})
    gcode='\n'.join(lines)+'\n'
    plan=FinalMotionPlan(identifier,tuple(new_samples),baseline.final_motion_plan.design_path,
                         baseline.final_motion_plan.printable_path,tuple(primitives))
    serial=SerializationValidator.check(gcode,mapping,new_samples,config)
    native=NativeArcValidator.check(gcode,primitives,config)
    semantic=semantic_signature(gcode)==semantic_signature(baseline.gcode)
    valid=serial['status']=='PASS' and native['status']=='PASS' and semantic
    counts=exported_metrics(gcode,config.sampling.preferred_min_block_length_mm)
    validation={'status':'PASS' if valid else 'FAIL','coverage_complete':valid,
                'kind':'CONTINUOUS_PARENT_BOUND_PLUS_LINEAR_INTERPOLATION_BOUND',
                'parent_reference_hash':snapshot.hash,'parent_xy_bound_mm':parent['xy_error_mm'],
                'added_interpolation_bound_mm':max_extra,'added_rounding_budget_mm':xyz_round,
                'xy_error_mm':parent['xy_error_mm']+max_extra+xyz_round,
                'C_error_deg':parent['C_error_deg']+c_round,'semantic_preserved':semantic,
                'serialization':serial,'native_check':native}
    report={'conversion_id':identifier,'status':validation['status'],'config':config.to_dict(),
            'validators':{'GlobalReference':validation},'reference_hash':snapshot.hash,
            'note':'G1 dense computational control; parent analytic primitives preserved within a composed bound.'}
    result=Result(new_samples,gcode,report,'',{'conversion_id':identifier,
                  'machine_primitives':[p.to_dict() for p in primitives],'samples':[s.to_dict() for s in new_samples]},plan)
    metrics={'feasible':valid,'status':validation['status'],'coverage_complete':valid,
             'reference_hash':snapshot.hash,'N_blocks':counts['motion_blocks'],'M_microblocks':counts['linear_deposition_microblocks'],
             'arc_microblocks':0,'bytes':counts['file_size_bytes'],'xy_error_mm':validation['xy_error_mm'],
             'C_error_deg':validation['C_error_deg'],'validation':validation,'t_sim_s':None,'t_print_real_s':None}
    return Evaluation(result,metrics)
