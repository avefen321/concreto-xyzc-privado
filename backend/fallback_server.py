"""Dependency-free local server for environments where FastAPI cannot be installed.

The documented and preferred API is backend.main (FastAPI). This adapter exposes
the same endpoints on localhost so the application remains usable offline.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
from pathlib import Path
import json
import mimetypes
import threading
from .core.config import Config
from .core.gcode import parse, interpret, linearize_arcs, motion_extent_points
from .core.layer_z import separate_layer_entries
from .core.layer_completion import review_source, prepare_layer_closure
from .core.cordon import prepare_cordon
from .core.path import build
from .core.conversion import convert

ROOT = Path(__file__).resolve().parent.parent
RESULTS = {}
LOCK = threading.Lock()


class Handler(BaseHTTPRequestHandler):
    def send(self, code, content, media_type='application/json', filename=None):
        if isinstance(content, (dict, list)):
            content = json.dumps(content, ensure_ascii=False, allow_nan=False).encode('utf-8')
        elif isinstance(content, str):
            content = content.encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', media_type + '; charset=utf-8')
        self.send_header('Content-Length', str(len(content)))
        if filename:
            self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self):
        path = urlparse(self.path).path
        if path=='/api/config':
            self.send(200,{'defaults':Config().to_dict(),'legacy':Config.legacy().to_dict()}); return
        files = {'/': ROOT / 'frontend' / 'index.html',
                 '/css/app.css': ROOT / 'frontend' / 'css' / 'app.css',
                 '/css/interactive.css': ROOT / 'frontend' / 'css' / 'interactive.css',
                 '/js/app.js': ROOT / 'frontend' / 'js' / 'app.js',
                 '/js/simulation.js': ROOT / 'frontend' / 'js' / 'simulation.js',
                 '/examples/cuadrado.ngc': ROOT / 'examples' / 'cuadrado.ngc'}
        files['/examples/contorno_organico.ngc']=ROOT / 'examples' / 'contorno_organico.ngc'
        if path in files:
            file = files[path]
            self.send(200, file.read_bytes(), mimetypes.guess_type(file)[0] or 'text/plain')
            return
        parts = path.strip('/').split('/')
        try:
            if len(parts) == 3 and parts[:2] == ['api', 'simulate']:
                result = RESULTS[parts[2]]
                self.send(200, result.simulation)
                return
            if len(parts) == 4 and parts[:2] == ['api', 'conversion']:
                result = RESULTS[parts[2]]
                if parts[3] == 'report':
                    self.send(200, result.report, filename='conversion_report.json'); return
                if parts[3] == 'samples':
                    self.send(200, result.csv_text, 'text/csv', 'samples.csv'); return
                if parts[3] == 'gcode':
                    if result.report['status'] == 'FAIL':
                        self.send(409, {'detail': 'Exportación bloqueada por auditoría'}); return
                    self.send(200, result.gcode, 'text/plain', 'converted_xyzc.ngc'); return
        except KeyError:
            pass
        self.send(404, {'detail': 'No encontrado'})

    def do_POST(self):
        path = urlparse(self.path).path
        if path not in ('/api/files/parse', '/api/files/closure-analysis', '/api/convert'):
            self.send(404, {'detail': 'No encontrado'}); return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if length > 30_000_000:
                self.send(413, {'detail': 'Archivo demasiado grande'}); return
            request = json.loads(self.rfile.read(length))
            source = request['source']
            config = Config.from_dict(request.get('config'))
            if path == '/api/files/closure-analysis':
                self.send(200, review_source(source, config)); return
            if path == '/api/files/parse':
                blocks = parse(source)
                motions, warnings = interpret(blocks)
                counts = {name: sum(1 for m in motions if m.kind == kind)
                          for name, kind in (('G0','RAPID'),('G1','LINEAR'),('G2','ARC_CW'),('G3','ARC_CCW'))}
                motions, closure_review, closure_warnings = prepare_layer_closure(motions, blocks, config)
                warnings.extend(closure_warnings)
                motions, cordon_review = prepare_cordon(motions, blocks, config, closure_review)
                motions, entry_warnings = separate_layer_entries(motions, config)
                warnings.extend(entry_warnings)
                strokes, layers, more = build(motions, config)
                points = [point for motion in motions for point in motion_extent_points(motion)]
                self.send(200, {'filename': request.get('filename', 'input.ngc'),
                                'bytes': len(source.encode('utf-8')), 'lines': len(blocks),
                                'counts': counts, 'layers': len(layers), 'strokes': len(strokes),
                                'ranges': {axis: [min(p[i] for p in points), max(p[i] for p in points)] if points else [0,0]
                                           for i, axis in enumerate('XYZ')},
                                'feeds_mm_min': sorted(set(m.feed for m in motions if m.feed is not None)),
                                'closure_review': closure_review,
                                'cordon_review': cordon_review,
                                'warnings': warnings + more}); return
            result = convert(source, config)
            key = result.report['conversion_id']
            with LOCK:
                RESULTS[key] = result
                while len(RESULTS) > 8:
                    RESULTS.pop(next(iter(RESULTS)))
            self.send(200, {'id': key, 'status': result.report['status'],
                            'report': result.report, 'sample_count': len(result.samples)})
        except (ValueError, KeyError, TypeError) as error:
            self.send(422, {'detail': str(error)})


def run(port=8000):
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    print(f'Servidor local sin dependencias: http://127.0.0.1:{port}')
    server.serve_forever()
