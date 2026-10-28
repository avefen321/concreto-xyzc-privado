"""Tokenizing and modal interpretation. Internal positions are always absolute mm."""
from dataclasses import dataclass, field
import math
import re

# Lowercase e is accepted as an exponent; uppercase E remains the extrusion word.
WORD = re.compile(r"([A-Za-z])\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?)")
SUPPORTED_G = {0, 1, 2, 3, 17, 18, 19, 20, 21, 90, 91, 90.1, 91.1, 92, 94, 61, 61.1, 64}
SUPPORTED_M = {2, 3, 4, 5, 30, 82, 83}


@dataclass
class Block:
    line: int
    raw: str
    words: list[tuple[str, float]]
    comments: list[str]
    warnings: list[str] = field(default_factory=list)

    def value(self, letter: str):
        values = [v for k, v in self.words if k == letter]
        return values[-1] if values else None


@dataclass
class Motion:
    line: int
    kind: str
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    feed: float | None
    deposition: bool
    block: Block
    e: float | None = None
    e_start: float | None = None
    e_end: float | None = None
    arc: dict = field(default_factory=dict)
    layer_id: int | None = None
    stroke_id: int | None = None
    original_kind: str | None = None
    barrier_id: int = 0
    source_motion_id: int | None = None
    layer_closure_role: str | None = None
    closure_reference_line: int | None = None
    cordon_role: str | None = None
    c_start: float = 0.0
    c_end: float = 0.0


def parse(text: str) -> list[Block]:
    blocks = []
    for number, raw in enumerate(text.splitlines(), 1):
        comments = []
        clean = []
        i = 0
        while i < len(raw):
            if raw[i] == ';':
                comments.append(raw[i:])
                break
            if raw[i] == '(':
                end = raw.find(')', i + 1)
                if end < 0:
                    raise ValueError(f"Línea {number}: comentario sin cerrar")
                comments.append(raw[i:end + 1])
                i = end + 1
            else:
                clean.append(raw[i]); i += 1
        code = ''.join(clean)
        words = []
        pos = 0
        for match in WORD.finditer(code):
            if code[pos:match.start()].strip():
                raise ValueError(f"Línea {number}: token inválido {code[pos:match.start()]!r}")
            value = float(match.group(2))
            if not math.isfinite(value):
                raise ValueError(f"Línea {number}: número no finito")
            words.append((match.group(1).upper(), value))
            pos = match.end()
        if code[pos:].strip():
            raise ValueError(f"Línea {number}: token inválido {code[pos:]!r}")
        blocks.append(Block(number, raw, words, comments))
    return blocks


def interpret(blocks: list[Block], *, initial_xyz=None) -> tuple[list[Motion], list[str]]:
    xyz = list(initial_xyz) if initial_xyz is not None else [0.0, 0.0, 0.0]
    if len(xyz)!=3 or not all(math.isfinite(v) for v in xyz):
        raise ValueError('Estado XYZ inicial inválido')
    offset = [0.0, 0.0, 0.0]
    e_position = 0.0
    e_offset = 0.0
    unit = 1.0
    absolute = True
    center_absolute = False
    c_position = c_offset = 0.0
    e_absolute = True
    mode = 0
    plane = 17
    feed = None
    deposition = False
    process_known = False
    extrusion_seen = False
    stopped = False
    motions = []
    warnings = []
    barrier_id = 0
    for block in blocks:
        if stopped:
            break
        g_codes = [v for k, v in block.words if k == 'G']
        m_codes = [v for k, v in block.words if k == 'M']
        if any(g in (93,95) for g in g_codes):
            raise ValueError(f'Línea {block.line}: se requiere feed G94 (mm/min); G93/G95 no implementado')
        if any(g in (5,5.1,5.2,5.3) for g in g_codes):
            raise ValueError(f'Línea {block.line}: splines G5/G5.1/G5.2/G5.3 no implementadas para XYZC')
        if (m_codes or any(g not in (0, 1, 2, 3) for g in g_codes) or block.value('S') is not None or
                (block.value('C') is not None and not any(block.value(k) is not None for k in 'XYZ')) or
                'LOCKED' in block.raw.upper() or
                block.value('E') is not None and not any(block.value(k) is not None for k in 'XYZ')):
            barrier_id += 1
        if any(g in (28, 28.1, 30, 30.1) for g in g_codes):
            raise ValueError(f"Línea {block.line}: G28/G30 requiere posición de máquina desconocida; conversión bloqueada")
        for g in g_codes:
            if g not in SUPPORTED_G:
                warnings.append(f"Línea {block.line}: G{g:g} no interpretado; preservado")
            elif g in (20, 21): unit = 25.4 if g == 20 else 1.0
            elif g in (90, 91): absolute = g == 90
            elif g in (90.1, 91.1): center_absolute = g == 90.1
            elif g in (17, 18, 19): plane = int(g)
            elif g in (0, 1, 2, 3): mode = int(g)
        for m in m_codes:
            if m not in SUPPORTED_M:
                warnings.append(f"Línea {block.line}: M{m:g} no interpretado; preservado")
            elif m in (3, 4): deposition = True; process_known = True
            elif m == 5: deposition = False; process_known = True
            elif m in (82, 83): e_absolute = m == 82
            elif m in (2, 30): stopped = True
        f = block.value('F')
        if f is not None:
            feed = f * unit
            if feed <= 0:
                raise ValueError(f"Línea {block.line}: feed no positivo")
        if 92 in g_codes:
            for j, letter in enumerate('XYZ'):
                value = block.value(letter)
                if value is not None:
                    offset[j] = xyz[j] - value * unit
            value = block.value('E')
            if value is not None: e_offset = e_position - value * unit
            if block.value('C') is not None: c_offset = c_position - block.value('C')
            continue
        target = xyz.copy()
        has_xyz = False
        for j, letter in enumerate('XYZ'):
            value = block.value(letter)
            if value is not None:
                has_xyz = True
                target[j] = value * unit + offset[j] if absolute else xyz[j] + value * unit
        e = block.value('E')
        e_start = e_position
        e_delta = None
        if e is not None:
            extrusion_seen = True
            e_target = e * unit + e_offset if e_absolute else e_position + e * unit
            e_delta = e_target - e_position
            e_position = e_target
        c_target=c_position
        if block.value('C') is not None:
            c_target=block.value('C')+c_offset if absolute else c_position+block.value('C')
        if not has_xyz and block.value('C') is not None and mode not in (2,3) and e is None:
            motions.append(Motion(block.line,'C_ONLY',tuple(xyz),tuple(xyz),feed,False,block,
                                  original_kind='C_ONLY',barrier_id=barrier_id,source_motion_id=len(motions)+1,
                                  c_start=c_position,c_end=c_target))
            c_position=c_target
            continue
        if not has_xyz and mode not in (2, 3):
            continue
        if not has_xyz and not any(block.value(k) is not None for k in ('I', 'J', 'K', 'R')):
            continue
        if mode in (2, 3) and plane != 17:
            raise ValueError(f"Línea {block.line}: solo arcos G17 XY están implementados")
        active = (deposition if process_known else
                  (e_delta is not None and e_delta > 1e-9) if extrusion_seen else mode != 0)
        kind = {0: 'RAPID', 1: 'LINEAR', 2: 'ARC_CW', 3: 'ARC_CCW'}[mode]
        arc = {k: block.value(k) * unit for k in ('I', 'J', 'K', 'R') if block.value(k) is not None}
        if mode in (2,3):
            if 'K' in arc: raise ValueError(f'Línea {block.line}: K no válido para arco G17')
            if 'R' in arc and any(k in arc for k in ('I','J')):
                raise ValueError(f'Línea {block.line}: arco mezcla R e I/J')
            if center_absolute and 'R' not in arc:
                if not all(k in arc for k in ('I','J')): raise ValueError(f'Línea {block.line}: G90.1 requiere I y J')
                arc['I'] += offset[0] - xyz[0]
                arc['J'] += offset[1] - xyz[1]
            arc['plane']=plane
            turns=block.value('P')
            if turns is not None:
                if turns < 1 or turns != int(turns): raise ValueError(f'Línea {block.line}: P de arco debe ser entero positivo')
                arc['turns']=int(turns)
            arc['center_mode']='G90.1' if center_absolute else 'G91.1'
        motions.append(Motion(block.line, kind, tuple(xyz), tuple(target), feed, active, block,
                              e, e_start if extrusion_seen else None, e_position if extrusion_seen else None,
                              arc, original_kind=kind, barrier_id=barrier_id, source_motion_id=len(motions)+1,
                              c_start=c_position,c_end=c_target))
        c_position=c_target
        xyz = target
    return motions, warnings


def arc_geometry(motion):
    """Resolve the existing G17 arc parser into one analytic primitive."""
    x0, y0, z0 = motion.start
    x1, y1, z1 = motion.end
    cw = motion.kind == 'ARC_CW'
    if 'R' in motion.arc:
        radius = abs(motion.arc['R'])
        dx, dy = x1 - x0, y1 - y0
        chord = math.hypot(dx, dy)
        if chord <= 1e-12 or chord > 2 * radius + 1e-8:
            raise ValueError(f"Línea {motion.line}: radio de arco inválido")
        h = math.sqrt(max(0, radius * radius - chord * chord / 4))
        sign = -1 if cw else 1
        if motion.arc['R'] < 0: sign *= -1
        cx = (x0 + x1) / 2 - sign * dy * h / chord
        cy = (y0 + y1) / 2 + sign * dx * h / chord
    else:
        if 'I' not in motion.arc and 'J' not in motion.arc:
            raise ValueError(f"Línea {motion.line}: arco sin I/J/R")
        cx = x0 + motion.arc.get('I', 0)
        cy = y0 + motion.arc.get('J', 0)
        radius = math.hypot(x0 - cx, y0 - cy)
        if abs(math.hypot(x1 - cx, y1 - cy) - radius) > 0.01:
            raise ValueError(f"Línea {motion.line}: extremo de arco fuera del radio")
    a0 = math.atan2(y0 - cy, x0 - cx)
    a1 = math.atan2(y1 - cy, x1 - cx)
    sweep = (a1 - a0) % (2 * math.pi)
    if cw: sweep = sweep - 2 * math.pi if sweep else -2 * math.pi
    elif not sweep: sweep = 2 * math.pi
    if radius<=0:
        raise ValueError(f'Línea {motion.line}: radio nulo')
    sweep += math.copysign(2*math.pi*(motion.arc.get('turns',1)-1),sweep)
    return cx, cy, radius, a0, sweep


def motion_extent_points(motion):
    """Exact endpoint/cardinal samples sufficient for XYZ axis bounds."""
    points=[motion.start,motion.end]
    if motion.kind not in ('ARC_CW','ARC_CCW'):
        return points
    cx,cy,radius,a0,sweep=arc_geometry(motion)
    magnitude=abs(sweep); direction=1 if sweep>0 else -1
    for angle in (0.,math.pi/2,math.pi,3*math.pi/2):
        distance=((angle-a0)%(2*math.pi) if direction>0 else
                  (a0-angle)%(2*math.pi))
        if distance<=magnitude+1e-12:
            fraction=distance/magnitude if magnitude else 0.
            points.append((cx+radius*math.cos(angle),cy+radius*math.sin(angle),
                           motion.start[2]+(motion.end[2]-motion.start[2])*fraction))
    return points


def linearize_arcs(motions: list[Motion], max_step_mm: float = 1.0,
                   max_turn_deg: float = 2.0, max_chord_error_mm: float = 0.03) -> list[Motion]:
    """Discretize G17 arcs, preserving endpoints and source-line traceability."""
    result = []
    for motion in motions:
        if motion.kind not in ('ARC_CW', 'ARC_CCW'):
            result.append(motion); continue
        cx, cy, radius, a0, sweep = arc_geometry(motion)
        x0, y0, z0 = motion.start
        x1, y1, z1 = motion.end
        ratio=min(1.,max_chord_error_mm/radius)
        chord_angle=2*math.asin(math.sqrt(ratio*(2-ratio)))
        steps = max(1, math.ceil(abs(sweep) * radius / max_step_mm),
                    math.ceil(abs(math.degrees(sweep))/max_turn_deg),
                    math.ceil(abs(sweep)/chord_angle))
        last = motion.start
        for i in range(1, steps + 1):
            t = i / steps
            point = motion.end if i == steps else (cx + radius * math.cos(a0 + sweep * t), cy + radius * math.sin(a0 + sweep * t), z0 + (z1 - z0) * t)
            ea = (motion.e_start + (motion.e_end - motion.e_start) * ((i - 1) / steps)
                  if motion.e_start is not None and motion.e_end is not None else None)
            eb = (motion.e_start + (motion.e_end - motion.e_start) * t
                  if motion.e_start is not None and motion.e_end is not None else None)
            result.append(Motion(motion.line, 'LINEAR', last, point, motion.feed, motion.deposition,
                                 motion.block, motion.e, ea, eb, {}, original_kind='LINEAR',
                                 barrier_id=motion.barrier_id, source_motion_id=motion.source_motion_id,
                                 layer_closure_role=motion.layer_closure_role,
                                 closure_reference_line=motion.closure_reference_line,
                                 cordon_role=motion.cordon_role))
            last = point
    return result
