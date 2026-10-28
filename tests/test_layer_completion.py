import csv
import io
import math
from pathlib import Path
import unittest
from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import parse, interpret
from backend.core.layer_completion import review_source, prepare_layer_closure


ROOT = Path(__file__).resolve().parent.parent


def repeated_contour(layers=3, closed=False, subdivide=False, arc=False):
    lines = ['G21', 'G90', 'G1 F2400 X50 Y0 Z0']
    for k in range(layers):
        if k: lines.append(f'G1 X50 Y0 Z{k*20}')
        points = [(100, 0), (100, 100), (0, 100), (0, 0), (30, 0)]
        if subdivide and k:
            points = [(75, 0), (100, 0), (100, 50), (100, 100), (50, 100),
                      (0, 100), (0, 50), (0, 0), (15, 0), (30, 0)]
        for x, y in points:
            if arc and (x, y) == (100, 100):
                lines.append(f'G3 X{x} Y{y} Z{k*20} I0 J50')
            else:
                lines.append(f'G1 X{x} Y{y} Z{k*20}')
        if closed: lines.append(f'G1 X50 Y0 Z{k*20}')
    return '\n'.join(lines+['M30'])+'\n'


def completion_config(**sections):
    return Config.from_dict({'layer_completion': {'decision': 'complete'}, **sections})


class LayerCompletionTests(unittest.TestCase):
    def test_real_files_close_every_layer_and_export_same_plan(self):
        for name, gap, seam in [('cruz_CA.gcode', 72., [402.64, 175.63]),
                                ('hexagono_suave_CA.gcode', 49.6105230773, [93.45, 381.86])]:
            with self.subTest(name=name):
                source = (ROOT/name).read_text()
                review = review_source(source, Config())
                self.assertTrue(review['review_required'])
                self.assertAlmostEqual(review['missing_first_layer_length_mm'], gap, places=6)
                self.assertEqual(review['seam_xy'], seam)
                r = convert(source, completion_config())
                self.assertNotEqual(r.report['status'], 'FAIL', r.report['errors'])
                c = r.report['layer_completion']
                self.assertEqual(c['status'], 'COMPLETED')
                self.assertEqual(c['reassigned_return_count'], 29)
                self.assertEqual(c['added_closure_count'], 1)
                self.assertEqual(c['validation']['status'], 'PASS')
                self.assertEqual(c['validation']['max_gap_mm'], 0.)
                self.assertEqual(c['validation']['max_transition_xy_mm'], 0.)
                self.assertEqual(len(c['validation']['layers']), 30)
                paths = [s for s in r.simulation['final_strokes'] if s['deposition']]
                self.assertEqual(len(paths), 30)
                for i, stroke in enumerate(paths):
                    self.assertEqual(stroke['points'][0], stroke['points'][-1])
                    self.assertEqual(stroke['points'][0], seam+[20.+18*i])
                exported, _ = interpret(parse(r.gcode))
                vertical = [m for m in exported if abs(m.end[2]-m.start[2]) > .001]
                # Exclude the initial travel to the first deposition height.
                for m in vertical[1:]:
                    self.assertEqual(m.start[:2], m.end[:2])
                    self.assertEqual(list(m.end[:2]), seam)
                emitted = [s for s in r.samples if s.emit]
                self.assertEqual(len(exported), len(emitted))
                rounding_budget = math.sqrt(3) * 0.5 * 10**(-r.report['config']['serialization']['xyz_decimals'])
                for m, s in zip(exported, emitted):
                    self.assertLessEqual(math.dist(m.end, (s.X, s.Y, s.Z)), rounding_budget + 1e-9)
                rows = list(csv.DictReader(io.StringIO(r.csv_text)))
                self.assertEqual(len(rows), len(r.samples))
                final = [s for s in r.samples if s.layer_closure_role == 'ADDED_FINAL_RETURN']
                self.assertTrue(final)
                self.assertIn(c['closure_geometry']['source_line'], final[-1].source_lines)

    def test_preserve_leaves_canonical_input_unchanged_and_warns(self):
        source = repeated_contour()
        blocks = parse(source); motions, _ = interpret(blocks)
        before = [(m.kind, m.start, m.end, m.feed, m.deposition) for m in motions]
        after, report, warnings = prepare_layer_closure(motions, blocks, Config())
        self.assertEqual(before, [(m.kind, m.start, m.end, m.feed, m.deposition) for m in after])
        self.assertEqual(report['status'], 'PRESERVED')
        self.assertTrue(warnings)
        result = convert(source)
        self.assertEqual(result.report['validators']['Layers']['status'], 'WARNING')
        self.assertTrue(any('FIRST_LAYER_OPEN' in w for w in result.report['warnings']))

    def test_closed_square_does_not_prompt_or_add_moves(self):
        source = repeated_contour(6, closed=True)
        blocks = parse(source); motions, _ = interpret(blocks)
        after, report, _ = prepare_layer_closure(motions, blocks, completion_config())
        self.assertIs(after, motions)
        self.assertEqual(report['status'], 'NOT_NEEDED')
        self.assertFalse(report['review_required'])
        self.assertEqual(report['added_closure_count'], 0)

    def test_different_collinear_density_is_same_ordered_geometry(self):
        review = review_source(repeated_contour(subdivide=True), Config())
        self.assertTrue(review['review_required'])
        self.assertNotEqual(convert(repeated_contour(subdivide=True), completion_config()).report['status'], 'FAIL')

    def test_native_curves_stay_native_until_sampling(self):
        source = repeated_contour(arc=True)
        self.assertTrue(review_source(source, Config())['review_required'])
        blocks = parse(source); motions, _ = interpret(blocks)
        after, report, _ = prepare_layer_closure(motions, blocks, completion_config())
        original = [(m.start, m.end, m.arc) for m in motions if m.kind == 'ARC_CCW']
        repaired = [(m.start, m.end, m.arc) for m in after if m.kind == 'ARC_CCW']
        self.assertEqual(original, repaired)
        self.assertEqual(report['closure_geometry']['kind'], 'LINEAR')
        r = convert(source, completion_config())
        self.assertNotEqual(r.report['status'], 'FAIL', r.report['errors'])
        self.assertTrue(r.simulation['arc_primitives'])

    def test_explicit_rapid_approach_is_not_reclassified(self):
        source = repeated_contour().replace('G1 F2400 X50 Y0 Z0', 'G0 F2400 X50 Y0 Z0')
        r = convert(source, completion_config())
        self.assertEqual(r.report['layer_completion']['status'], 'COMPLETED')
        self.assertIsNone(r.report['layer_completion']['inferred_approach_source_line'])

    def test_closed_final_return_precedes_terminal_and_preserves_feed(self):
        source = repeated_contour().replace('G1 X30 Y0 Z40', 'G1 F1200 X30 Y0 Z40 ; final contour')
        r = convert(source, completion_config())
        blocks = parse(r.gcode)
        self.assertEqual(blocks[-1].value('M'), 30)
        motions, _ = interpret(blocks)
        self.assertEqual(motions[-1].end, (50., 0., 56.))
        self.assertEqual(motions[-1].feed, 1200.)
        self.assertEqual(r.report['validators']['Serialization']['status'], 'PASS')
        self.assertIn('; final contour', r.gcode)

    def test_repeat_count_geometry_direction_events_and_ramps_guarded(self):
        source = repeated_contour()
        cases = [repeated_contour(2),
                 source.replace('G1 X100 Y100 Z20', 'G1 X110 Y100 Z20'),
                 source.replace('G1 X100 Y0 Z20', 'G1 X0 Y100 Z20'),
                 source.replace('G1 X50 Y0 Z20', 'G0 X50 Y0 Z20'),
                 source.replace('G1 X50 Y0 Z20', 'G1 X30 Y0 Z20'),
                 source.replace('G1 X100 Y100 Z20', 'G1 X100 Y100 Z25'),
                 'M3\n'+source,
                 source.replace('G1 X100 Y100 Z20', 'G1 X100 Y100 Z20 E1'),
                 source.replace('G1 X100 Y100 Z20', 'G1 X100 Y100 Z20 (LOCKED)'),
                 source.replace('G1 X100 Y100 Z20', 'G4 P1\nG1 X100 Y100 Z20')]
        for i, case in enumerate(cases):
            with self.subTest(i=i):
                self.assertFalse(review_source(case, Config())['review_required'])
                blocks = parse(case); motions, _ = interpret(blocks)
                output, _, _ = prepare_layer_closure(motions, blocks, completion_config())
                self.assertIs(output, motions)

    def test_original_z_schedule_and_safe_seam_remain_supported(self):
        for settings in [{'z_conversion': {'enabled': False}}, {'layer_transition': {'safe_seam': True}}]:
            r = convert(repeated_contour(), completion_config(**settings))
            self.assertNotEqual(r.report['status'], 'FAIL', r.report['errors'])
            self.assertEqual(r.report['layer_completion']['validation']['max_gap_mm'], 0.)

    def test_decisions_have_distinct_deterministic_ids(self):
        source = repeated_contour()
        a = convert(source, completion_config())
        b = convert(source, completion_config())
        c = convert(source)
        self.assertEqual(a.gcode, b.gcode)
        self.assertEqual(a.report, b.report)
        self.assertNotEqual(a.report['conversion_id'], c.report['conversion_id'])

    def test_invalid_completion_config_rejected(self):
        for data in [{'decision': 'auto'}, {'min_reference_layers': 1},
                     {'min_reference_layers': 2.5}, {'match_tolerance_mm': 0}]:
            with self.assertRaises(ValueError): Config.from_dict({'layer_completion': data})


if __name__ == '__main__': unittest.main()
