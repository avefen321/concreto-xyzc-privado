"""Independent checks over the single final dataset and its serialized records."""
import math
from .gcode import parse, interpret, arc_geometry


class SyntaxValidator:
    name='Syntax'
    @staticmethod
    def check(gcode):
        try: parse(gcode); return {'status':'PASS'}
        except ValueError as error: return {'status':'FAIL','detail':str(error)}


class SerializationValidator:
    name='Serialization'
    @staticmethod
    def check(gcode,mapping,samples,config):
        emitted=[s for s in samples if s.emit]
        rows=[row for row in mapping if row['sample_id'] is not None]
        errors=[]; max_xyz=max_c=max_f=0.
        if [s.sample_id for s in emitted]!=[r['sample_id'] for r in rows]:
            errors.append('El orden serializado difiere del FinalMotionPlan')
        blocks=parse(gcode)
        roundtrip,_=interpret(blocks)
        motion_by_line={motion.line:motion for motion in roundtrip}
        by_id={s.sample_id:s for s in emitted}
        for row in rows:
            s=by_id[row['sample_id']]; b=blocks[row['output_line']-1]
            motion=motion_by_line.get(row['output_line'])
            if motion is None:
                errors.append(f"Línea {row['output_line']}: bloque serializado sin movimiento")
                continue
            if s.motion_type!='C_ONLY':
                # Compare the modal endpoint reconstructed by the interpreter.
                # Full I/J circles intentionally omit X and Y in LinuxCNC.
                max_xyz=max(max_xyz,math.dist((s.X,s.Y,s.Z),motion.end))
            max_c=max(max_c,abs(motion.c_end-s.C_deg))
            if b.value('F') is not None and s.feed_mm_min is not None:
                max_f=max(max_f,abs(b.value('F')-s.feed_mm_min))
        xyz_budget=math.sqrt(3)*0.5*10**(-config.serialization.xyz_decimals)+1e-9
        c_budget=0.5*10**(-config.serialization.c_decimals)+1e-9
        feed_budget=0.5*10**(-config.serialization.feed_decimals)+1e-9
        if max_xyz>xyz_budget or max_c>c_budget or max_f>feed_budget:
            errors.append('Redondeo supera los decimales de salida configurados')
        return {'status':'FAIL' if errors else 'PASS','errors':errors,'mapped_motion_count':len(rows),
                'max_xyz_rounding_error_mm':max_xyz,'max_c_rounding_error_deg':max_c,'max_feed_rounding_error_mm_min':max_f,
                'xyz_rounding_budget_mm':xyz_budget,'c_rounding_budget_deg':c_budget,
                'feed_rounding_budget_mm_min':feed_budget}


class NativeArcValidator:
    @staticmethod
    def check(gcode,primitives,config):
        errors=[]; count=0; center_error=radius_error=sweep_error=c_error=0.; reconstructed_pose_error=0.
        try:
            motions,_=interpret(parse(gcode))
            expected=list(primitives)
            if len(motions)!=len(expected): errors.append('Round-trip: cantidad de movimientos diferente')
            for m,p in zip(motions,expected):
                if p.metadata.get('reference_model')=='PARAMETRIC_HERMITE':
                    from .orientation import nearest_equivalent_angle
                    if p.type=='ARC':
                        cx,cy,_,_,_=arc_geometry(m)
                        sign=1 if m.kind=='ARC_CCW' else -1
                        targets=[math.degrees(math.atan2(q[1]-cy,q[0]-cx))+sign*90+config.orientation_offset_deg
                                 for q in (m.start,m.end)]
                    else:
                        if math.dist(m.start[:2],m.end[:2])<1e-12:
                            errors.append(f'{p.id}: redondeo colapsa un G1 reconstruido'); continue
                        targets=[math.degrees(math.atan2(m.end[1]-m.start[1],m.end[0]-m.start[0]))+config.orientation_offset_deg]*2
                    pose_reference_shift=p.metadata.get('c_reference_shift_deg',0.0)
                    for actual,target in zip((m.c_start,m.c_end),targets):
                        expected=nearest_equivalent_angle(target+pose_reference_shift,actual,
                                                          config.orientation.symmetry_period_deg)
                        reconstructed_pose_error=max(reconstructed_pose_error,abs(actual-expected))
                if p.type!='ARC': continue
                if not m.kind.startswith('ARC'): errors.append(f'{p.id}: perdió tipo ARC'); continue
                count+=1
                cx,cy,r,a,sweep=arc_geometry(m)
                center_error=max(center_error,math.dist((cx,cy),p.center_xyz[:2]))
                radius_error=max(radius_error,abs(r-p.radius))
                sweep_error=max(sweep_error,abs(math.degrees(sweep-p.sweep_rad)))
                c_error=max(c_error,abs(m.c_end-p.end_c))
                if abs(m.c_start-p.start_c)>10**(-config.serialization.c_decimals)+1e-8:
                    errors.append(f'{p.id}: C inicial serializado difiere de la primitiva')
                if math.copysign(1,sweep)!=math.copysign(1,p.sweep_rad): errors.append(f'{p.id}: sentido cambiado')
                if p.process_state=='DEPOSIT' and abs((p.end_c-p.start_c)-math.degrees(p.sweep_rad))>1e-6:
                    errors.append(f'{p.id}: C no sigue barrido analítico')
                if p.feed is None: errors.append(f'{p.id}: arco sin F conocido')
            xyz_budget=2*math.sqrt(2)*10**(-config.serialization.xyz_decimals)+2*10**(-config.serialization.ijk_decimals)
            if center_error>xyz_budget or radius_error>xyz_budget: errors.append('Round-trip: error de centro/radio excesivo')
            tolerance=min(config.arc_fit.orientation_tolerance_deg,config.arc_fit.tangent_tolerance_deg,
                          config.sampling.max_orientation_interp_error_deg,config.simplification.orientation_tolerance_deg)
            if reconstructed_pose_error>tolerance+1e-7:
                errors.append('Orientación del G-code reconstruido redondeado supera tolerancia; aumente decimales XYZ/C/IJ')
        except ValueError as error: errors.append(str(error))
        return {'status':'FAIL' if errors else 'PASS','errors':errors,'arc_count':count,
                'max_center_error_mm':center_error,'max_radius_error_mm':radius_error,
                'max_sweep_error_deg':sweep_error,'max_c_error_deg':c_error,
                'max_reconstructed_export_pose_error_deg':reconstructed_pose_error}


class SamplingValidator:
    name='Sampling'
    @staticmethod
    def check(samples,config):
        previous={}; max_delta=max_length=max_error=max_chord=0.
        for s in samples:
            if not s.deposition_active: continue
            prior=previous.get(s.stroke_id)
            if prior and s.motion_type!='C_ONLY':
                delta=abs(s.C_deg-prior.C_deg)
                length=math.dist((s.X,s.Y,s.Z),(prior.X,prior.Y,prior.Z))
                max_delta=max(max_delta,delta)
                if config.sampling.mode=='legacy' or s.primitive_type!='STRAIGHT': max_length=max(max_length,length)
                if s.primitive_type=='ARC' and s.curvature_1_mm:
                    radius=1/abs(s.curvature_1_mm)
                    max_chord=max(max_chord,radius*(1-math.cos(math.radians(delta)/2)))
            max_error=max(max_error,s.orientation_error_deg)
            previous[s.stroke_id]=s
        passed=(max_delta<=config.sampling.max_delta_c_deg+1e-7 and max_length<=config.sampling.max_linear_step_mm+1e-7
                and max_error<=config.sampling.max_orientation_interp_error_deg+1e-7
                and max_chord<=config.corner_smoothing.max_chord_error_mm+1e-7)
        max_geometry=max((s.geometry_error_mm for s in samples if s.primitive_type!='ARC'),default=0.)
        if config.sampling.mode!='legacy':
            passed=passed and max_geometry<=min(config.sampling.geometry_error_mm,config.simplification.geometry_tolerance_mm)+1e-7
        return {'status':'PASS' if passed else 'FAIL','max_delta_c_deg':max_delta,'max_linear_step_mm':max_length,
                'max_orientation_error_deg':max_error,'max_chord_error_mm':max_chord,'max_geometry_error_mm':max_geometry}


class OrientationValidator:
    name='Orientation'
    @staticmethod
    def check(samples,config):
        unit=physical=0.
        for s in samples:
            if s.tangent_xy is None: continue
            unit=max(unit,abs(math.hypot(*s.tangent_xy)-1),abs(math.hypot(*s.nozzle_vector_xy)-1))
            physical_angle=s.C_deg-s.c_reference_shift_deg
            physical=max(physical,abs(s.nozzle_vector_xy[0]-math.cos(math.radians(physical_angle))),
                         abs(s.nozzle_vector_xy[1]-math.sin(math.radians(physical_angle))))
        return {'status':'PASS' if max(unit,physical)<1e-8 else 'FAIL','max_unit_vector_error':unit,'max_pose_vector_error':physical}


class InitialAlignmentValidator:
    name='InitialAlignment'
    @staticmethod
    def check(report,primitives,gcode,config):
        reference=report.get('initial_c_reference',{})
        if reference.get('mode')!='ROTATE_AT_START':
            return {'status':'NOT CONFIGURED','mode':reference.get('mode')}
        alignments=[p for p in primitives if p.generated_reason=='INITIAL_C_ALIGNMENT']
        errors=[]
        if reference.get('required'):
            if len(alignments)!=1:
                errors.append('Debe existir exactamente una alineación C inicial')
            else:
                p=alignments[0]
                if p.type!='C_ONLY' or p.process_state!='TRAVEL':
                    errors.append('La alineación inicial no es un C-only con proceso inactivo')
                if math.dist(p.start_xyz,p.end_xyz)>config.precision.geometry_epsilon:
                    errors.append('La alineación inicial modifica XYZ')
                if abs(p.end_c-reference['target_c_deg'])>config.precision.angle_epsilon_deg:
                    errors.append('La alineación inicial no alcanza el C objetivo')
                first_deposition=next((i for i,q in enumerate(primitives)
                                       if q.process_state=='DEPOSIT'),None)
                if first_deposition is not None and primitives.index(p)>=first_deposition:
                    errors.append('La alineación inicial ocurre después de comenzar la deposición')
                lines=gcode.splitlines()
                c_line=next((i for i,line in enumerate(lines)
                             if line.startswith('G1 C') and
                             f"C{p.end_c:.{config.serialization.c_decimals}f}" in line),None)
                if c_line is None:
                    errors.append('Bloque C-only inicial ausente del G-code')
                elif c_line==0 or not lines[c_line-1].startswith('G61.1'):
                    errors.append('Falta parada exacta G61.1 antes de orientar C')
                elif c_line+1>=len(lines) or not lines[c_line+1].startswith(('G64','G61')):
                    errors.append('No se restauró el modo de trayectoria después de orientar C')
        elif alignments:
            errors.append('Se generó un giro inicial redundante')
        return {'status':'FAIL' if errors else 'PASS','errors':errors,
                'alignment_count':len(alignments),'xyz_held':not errors or reference.get('xyz_held')}


class GeometryValidator:
    name='Geometry'
    @staticmethod
    def check(report): return dict(report['geometry'])


class CornerValidator:
    name='Corners'
    @staticmethod
    def check(report):
        statuses=[f['status'] for f in report['fillets']]+[c['status'] for c in report['corners']]
        return {'status':'FAIL' if 'FAIL' in statuses else 'WARNING' if any(s!='PASS' for s in statuses) else 'PASS'}


class CurveValidator:
    name='Curves'
    @staticmethod
    def check(sampling): return {'status':sampling['status'],'max_orientation_interp_error_deg':sampling['max_orientation_error_deg']}


class LayerValidator:
    name='Layers'
    @staticmethod
    def check(report):
        closure = report.get('layer_completion', {})
        return {'status':'FAIL' if any(x['status']=='FAIL' for x in report['layers']) or closure.get('validation', {}).get('status') == 'FAIL' else
                'WARNING' if any(x['status']=='WARNING' for x in report['safe_seams']) or closure.get('status') == 'PRESERVED' else 'PASS',
                'closure_status': closure.get('status', 'NOT_APPLICABLE'),
                'max_layer_xy_drift_mm':report['geometry']['max_interlayer_xy_drift_mm']}


class KinematicValidator:
    name='Kinematics'
    @staticmethod
    def check(report): return dict(report['kinematics'])


class SemanticValidator:
    name='Semantic'
    @staticmethod
    def check(report): return {'status':'WARNING' if report['warnings'] else 'PASS'}


class ReconstructionValidator:
    @staticmethod
    def check(primitives,config):
        errors=[]; count=0
        geometry=min(config.arc_fit.geometry_tolerance_mm,config.sampling.geometry_error_mm,
                     config.simplification.geometry_tolerance_mm)
        angle=min(config.arc_fit.orientation_tolerance_deg,config.arc_fit.tangent_tolerance_deg,
                  config.sampling.max_orientation_interp_error_deg,config.simplification.orientation_tolerance_deg)
        for p in primitives:
            m=p.metadata
            if m.get('reference_model')!='PARAMETRIC_HERMITE': continue
            count+=1
            values=[m.get(k,float('nan')) for k in ('max_fit_error_mm','max_orientation_fit_error_deg',
                                                    'reconstruction_error_mm','approximation_error_mm')]
            if not all(math.isfinite(v) and v>=0 for v in values):
                errors.append(f'{p.id}: certificado no finito'); continue
            if (values[0]+m.get('output_rounding_geometry_budget_mm',0.)>geometry+1e-8 or
                    values[1]>angle+1e-8 or values[2]+values[3]>geometry+1e-8):
                errors.append(f'{p.id}: certificado excede el contrato de reconstrucción')
            if m.get('certificate') not in ('POLYNOMIAL_RADIAL_AND_TANGENT_CONE','BEZIER_HULL_AND_TANGENT_CONE'):
                errors.append(f'{p.id}: certificado ausente')
            if p.type=='LINE' and m.get('max_line_pose_error_deg',float('inf'))>angle+1e-8:
                errors.append(f'{p.id}: C se separa de la dirección del G1')
        return {'status':'FAIL' if errors else 'PASS','certified_primitives':count,'errors':errors,
                'orientation_reference':'PARAMETRIC_TANGENT','geometry_tolerance_mm':geometry,
                'orientation_tolerance_deg':angle}


def validate_final(samples,gcode,mapping,report,config,primitives=()):
    sampling=SamplingValidator.check(samples,config)
    checks={'Syntax':SyntaxValidator.check(gcode),'Semantic':SemanticValidator.check(report),
            'Geometry':GeometryValidator.check(report),'Corners':CornerValidator.check(report),
            'Curves':CurveValidator.check(sampling),'Layers':LayerValidator.check(report),
            'Orientation':OrientationValidator.check(samples,config),'Kinematics':KinematicValidator.check(report),
            'Sampling':sampling,'Serialization':SerializationValidator.check(gcode,mapping,samples,config),
            'InitialAlignment':InitialAlignmentValidator.check(report,primitives,gcode,config)}
    if report['orientation']['status']=='FAIL': checks['Orientation']['status']='FAIL'
    if primitives: checks['NativeArcs']=NativeArcValidator.check(gcode,primitives,config)
    if report.get('curve_reconstruction',{}).get('applied'):
        checks['CurveReconstruction']=ReconstructionValidator.check(primitives,config)
    return checks
