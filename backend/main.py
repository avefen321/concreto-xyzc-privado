"""Local FastAPI façade. Geometry and planning remain in backend.core."""
from pathlib import Path
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, PlainTextResponse, JSONResponse
from pydantic import BaseModel
from .core.config import Config
from .core.gcode import parse, interpret, linearize_arcs, motion_extent_points
from .core.layer_z import separate_layer_entries
from .core.layer_completion import review_source, prepare_layer_closure
from .core.cordon import prepare_cordon
from .core.path import build
from .core.conversion import convert, Result
import asyncio

ROOT = Path(__file__).resolve().parent.parent
app = FastAPI(title="Concrete CNC XYZC", version="3.7.0")
_results: dict[str, Result] = {}


class SourceRequest(BaseModel):
    source: str
    filename: str = "input.ngc"
    config: dict | None = None


def _result(conversion_id: str) -> Result:
    result = _results.get(conversion_id)
    if not result:
        raise HTTPException(404, "Conversión no encontrada")
    return result


@app.get("/")
def index():
    return FileResponse(ROOT / "frontend" / "index.html")


@app.get('/api/config')
def default_config():
    return {'defaults':Config().to_dict(),'legacy':Config.legacy().to_dict()}


@app.get("/css/{filename}")
def css(filename: str):
    if filename not in ("app.css", "interactive.css"): raise HTTPException(404)
    return FileResponse(ROOT / "frontend" / "css" / filename)


@app.get("/js/{filename}")
def js(filename: str):
    if filename not in ("app.js", "simulation.js"): raise HTTPException(404)
    return FileResponse(ROOT / "frontend" / "js" / filename, media_type="text/javascript")


@app.get("/examples/cuadrado.ngc")
def example_square():
    return FileResponse(ROOT / "examples" / "cuadrado.ngc", media_type="text/plain")


@app.get("/examples/contorno_organico.ngc")
def example_organic():
    return FileResponse(ROOT / "examples" / "contorno_organico.ngc", media_type="text/plain")


@app.post("/api/files/parse")
async def parse_file(request: SourceRequest):
    try:
        config = Config.from_dict(request.config)
        blocks = parse(request.source)
        motions, warnings = interpret(blocks)
        counts = {name: sum(1 for m in motions if m.kind == kind)
                  for name, kind in (("G0", "RAPID"), ("G1", "LINEAR"),
                                     ("G2", "ARC_CW"), ("G3", "ARC_CCW"))}
        motions, closure_review, closure_warnings = prepare_layer_closure(motions, blocks, config)
        warnings.extend(closure_warnings)
        motions, cordon_review = prepare_cordon(motions, blocks, config, closure_review)
        motions, entry_warnings = separate_layer_entries(motions, config)
        warnings.extend(entry_warnings)
        strokes, layers, more = build(motions, config)
        points = [point for motion in motions for point in motion_extent_points(motion)]
        return {"filename": request.filename, "bytes": len(request.source.encode('utf-8')),
                "lines": len(blocks), "counts": counts, "layers": len(layers), "strokes": len(strokes),
                "ranges": {axis: [min(p[i] for p in points), max(p[i] for p in points)] if points else [0, 0]
                           for i, axis in enumerate('XYZ')},
                "feeds_mm_min": sorted(set(m.feed for m in motions if m.feed is not None)),
                "closure_review": closure_review,
                "cordon_review": cordon_review,
                "warnings": warnings + more}
    except ValueError as error:
        raise HTTPException(422, str(error)) from error


@app.post('/api/files/closure-analysis')
async def closure_analysis(request: SourceRequest):
    try:
        return review_source(request.source, Config.from_dict(request.config))
    except ValueError as error:
        raise HTTPException(422, str(error)) from error


@app.post("/api/convert")
async def convert_file(request: SourceRequest):
    try:
        config = Config.from_dict(request.config)
        result = await asyncio.to_thread(convert, request.source, config)
    except ValueError as error:
        raise HTTPException(422, str(error)) from error
    conversion_id = result.report['conversion_id']
    _results[conversion_id] = result
    # Bounded in-memory storage for a local single-user app.
    while len(_results) > 8:
        _results.pop(next(iter(_results)))
    return {"id": conversion_id, "status": result.report['status'],
            "report": result.report, "sample_count": len(result.samples)}


@app.get("/api/simulate/{conversion_id}")
def simulate(conversion_id: str):
    return _result(conversion_id).simulation


@app.get("/api/conversion/{conversion_id}/report")
def report(conversion_id: str):
    return JSONResponse(_result(conversion_id).report,
                        headers={"Content-Disposition": 'attachment; filename="conversion_report.json"'})


@app.get("/api/conversion/{conversion_id}/gcode")
def gcode(conversion_id: str):
    result = _result(conversion_id)
    if result.report['status'] == 'FAIL':
        raise HTTPException(409, "Exportación bloqueada por auditoría")
    return PlainTextResponse(result.gcode,
                             headers={"Content-Disposition": 'attachment; filename="converted_xyzc.ngc"'})


@app.get("/api/conversion/{conversion_id}/samples")
def samples(conversion_id: str):
    return PlainTextResponse(_result(conversion_id).csv_text, media_type="text/csv",
                             headers={"Content-Disposition": 'attachment; filename="samples.csv"'})
