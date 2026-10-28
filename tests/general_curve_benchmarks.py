"""Reproducible general-curve comparisons and future optimization evaluations."""
import json
import math
from pathlib import Path
import time
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import parse,interpret,arc_geometry
from tests.general_curve_fixtures import cases
from tests.native_arc_benchmarks import fixtures


def summarize(result,seconds):
    r=result.report; n=r['native_arcs']; q=r['curve_reconstruction']
    applied=q['applied']
    return {'status':r['status'],'errors':r['errors'],'input_hash':r['input_hash'],
            'config':r['config'],'motion_blocks':len(result.final_motion_plan.primitives),
            'arcs':n['native_arc_count'],'fitted_arcs':n['fitted_arc_count'],
            'arc_length_fraction':n['native_arc_length_fraction'],'coverage_by_stroke':n['coverage_by_stroke'],
            'file_size_bytes':len(result.gcode.encode('utf-8')),'conversion_time_s':seconds,
            'certificate_scope':'RECONSTRUCTED_SMOOTH_SEGMENTS' if applied else None,
            'max_geometry_certificate_mm':q['max_certified_geometry_error_mm'] if applied else None,
            'max_orientation_certificate_deg':q['max_certified_orientation_error_deg'] if applied else None,
            'max_export_pose_error_deg':r['validators'].get('NativeArcs',{}).get('max_reconstructed_export_pose_error_deg') if applied else None,
            'max_arc_fit_error_reported_mm':n['max_fit_error_mm'],
            'max_orientation_error_reported_deg':r['metrics']['max_orientation_error_deg'],
            'max_required_c_velocity_deg_s':r['kinematics']['max_omega_c_deg_s'],
            'machine_mode_verified':r['linuxcnc']['machine_mode_verified'],'measured_execution_time_s':None,
            'reference_model':q['orientation_reference'],'source_function_recovered':False}


def preview(results,source_by_name,directory):
    """Draw actual parsed output arcs, not guessed SVG curves from vertices."""
    from PIL import Image,ImageDraw,ImageFont
    image=Image.new('RGB',(1600,1220),'#f7f5ef'); draw=ImageDraw.Draw(image)
    def font(size,bold=False):
        try: return ImageFont.truetype('C:/Windows/Fonts/segoeuib.ttf' if bold else 'C:/Windows/Fonts/segoeui.ttf',size)
        except OSError: return ImageFont.load_default()
    draw.text((55,32),'Curvas generales: recorrido exportado con I/J',font=font(37,True),fill='#183c40')
    draw.text((55,87),'Gris: entrada · Verde: G2/G3 · Rojo: G1 · Primera capa de cada ejemplo',font=font(22),fill='#50676a')
    chosen=[('organic_60_ccw','Contorno orgánico · 60 segmentos'),('organic_180_ccw','Contorno orgánico · 180 segmentos'),
            ('four_lobes_120','Cuatro lóbulos · cambios de curvatura'),('figure_eight','Figura de ocho · cruce y cierre')]
    for index,(name,title) in enumerate(chosen):
        result=results[name]; x0=45+(index%2)*780; y0=145+(index//2)*520
        draw.rounded_rectangle((x0,y0,x0+755,y0+490),radius=18,fill='white',outline='#cdd8d5',width=2)
        draw.text((x0+22,y0+15),title,font=font(25,True),fill='#183c40')
        draw.text((x0+22,y0+52),f"{result.report['native_arcs']['native_arc_count']} arcos · {result.report['native_arcs']['native_arc_length_fraction']:.1%} del recorrido en arcos",font=font(19),fill='#50676a')
        raw,_=interpret(parse(source_by_name[name])); original=[m for m in raw if m.deposition and m.kind!='RAPID']
        source=[original[0].start]+[m.end for m in original]
        minx=min(p[0] for p in source); maxx=max(p[0] for p in source)
        miny=min(p[1] for p in source); maxy=max(p[1] for p in source)
        scale=min(685/max(maxx-minx,1.),340/max(maxy-miny,1.))
        cx=x0+377.5; cy=y0+285
        def xy(p): return (cx+(p[0]-(minx+maxx)/2)*scale,cy-(p[1]-(miny+maxy)/2)*scale)
        draw.line([xy(p) for p in source],fill='#c5cbcb',width=5)
        motions,_=interpret(parse(result.gcode))
        for m,p in zip(motions,result.final_motion_plan.primitives):
            if p.process_state!='DEPOSIT' or p.layer_id!=1: continue
            if m.kind.startswith('ARC'):
                cxarc,cyarc,r,a,sweep=arc_geometry(m); steps=max(3,math.ceil(abs(sweep)/.025))
                points=[(cxarc+r*math.cos(a+sweep*i/steps),cyarc+r*math.sin(a+sweep*i/steps)) for i in range(steps+1)]
                color='#087e6c'
            else: points=[m.start,m.end]; color='#c75c4c'
            draw.line([xy(p) for p in points],fill=color,width=3)
        for p in source:
            x,y=xy(p); draw.ellipse((x-2,y-2,x+2,y+2),fill='#869b99')
    draw.text((55,1190),'Ejemplos geométricos representativos; no son una reconstrucción dimensional de la fotografía.',font=font(18),fill='#50676a')
    image.save(directory/'comparacion_curvas_generales.png')


def run():
    directory=Path('resultados_curvas_generales_2026-10-19'); directory.mkdir(exist_ok=True)
    dataset=list(cases())+[(name,source) for name,source in fixtures() if name in ('fillet_R30','square_R30','circle_360','spiral')]
    summaries={}; evaluations=[]; previews={}; sources=dict(dataset)
    for name,source in dataset:
        summaries[name]={}
        for model,label in (('POLYLINE_C','compatible'),('PARAMETRIC_HERMITE','parametric')):
            cfg=Config.from_dict({'arc_fit':{'reference_model':model},
                                 'serialization':{'xyz_decimals':6,'c_decimals':6,'ijk_decimals':6}})
            started=time.perf_counter(); result=convert(source,cfg); elapsed=time.perf_counter()-started
            row=summarize(result,elapsed); summaries[name][label]=row
            print(name,label,row['status'],row['motion_blocks'],'blocks',row['arcs'],'arcs',round(row['arc_length_fraction'],4),flush=True)
            if row['status']=='FAIL': raise RuntimeError(row['errors'])
            (directory/f'{name}_{label}.ngc').write_text(result.gcode,encoding='utf-8',newline='\n')
            (directory/f'{name}_{label}_report.json').write_text(json.dumps(result.report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
            evaluations.append({'case':name,'model':model,'parameters':cfg.to_dict(),
                'feasible':row['status']!='FAIL','feasibility_scope':'SOFTWARE_VALIDATORS_ONLY',
                'orientation_reference':row['reference_model'],'certificate_scope':row['certificate_scope'],
                'objectives':{key:row[key] for key in
                ('motion_blocks','arc_length_fraction','max_geometry_certificate_mm','max_orientation_certificate_deg',
                 'max_export_pose_error_deg','max_required_c_velocity_deg_s')},
                'reference_truth':'SOURCE_POLYLINE_AND_DECLARED_MODEL','original_function_error':None,
                'machine_execution_verified':False,'optimizer_run':False})
            if label=='parametric' and name in ('organic_60_ccw','organic_180_ccw','four_lobes_120','figure_eight'):
                previews[name]=result
    Path('GENERAL_CURVES_BENCHMARK_RESULTS_2026-10-19.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    (directory/'optimization_evaluations.json').write_text(json.dumps(evaluations,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    (directory/'parametric_config.json').write_text(json.dumps({'arc_fit':{'reference_model':'PARAMETRIC_HERMITE'},
        'serialization':{'xyz_decimals':6,'c_decimals':6,'ijk_decimals':6}},indent=2),encoding='utf-8')
    preview(previews,sources,directory)
    return summaries


if __name__=='__main__': run()
