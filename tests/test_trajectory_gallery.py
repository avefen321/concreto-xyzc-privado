"""Integration checks for the geometry-only test gallery."""
import math
from pathlib import Path
import unittest

from backend.core.config import Config
from backend.core.conversion import convert
from backend.core.gcode import arc_geometry, interpret, parse
from backend.core.simplification import distance_to_chord


GALLERY = Path(__file__).resolve().parent.parent / "codigos g testeo"
CONFIG = Config.from_dict({
    "arc_fit": {"reference_model": "PARAMETRIC_HERMITE"},
    "serialization": {"xyz_decimals": 6, "c_decimals": 6, "ijk_decimals": 6},
    "cordon": {"mode": "omit"},
    "z_conversion": {"enabled": False},
})


class TrajectoryGalleryTests(unittest.TestCase):
    def test_each_path_converts_and_round_trips_without_process_commands(self):
        sources = sorted(GALLERY.glob("[0-9][0-9]_*.ngc"))
        self.assertEqual(len(sources), 15)
        for file in sources:
            with self.subTest(file=file.name):
                source = file.read_text(encoding="ascii")
                raw, _ = interpret(parse(source))
                result = convert(source, CONFIG)
                self.assertNotEqual(result.report["status"], "FAIL", result.report["errors"])
                self.assertEqual(len(result.report["layers"]), 3)
                self.assertEqual(result.report["z_conversion"]["target_layer_z_values"],
                                 [20., 38., 56.])
                self.assertEqual(result.report["validators"]["NativeArcs"]["status"], "PASS")
                output, _ = interpret(parse(result.gcode))
                self.assertEqual(len(output), len(result.final_motion_plan.primitives))
                for motions in (raw, output):
                    rises = [m for m in motions if m.kind == "LINEAR" and
                             math.dist(m.start[:2], m.end[:2]) < 1e-7 and
                             m.end[2] > m.start[2]]
                    self.assertGreaterEqual(len(rises), 2)
                self.assertFalse(any(letter in {"E", "S", "T"} for block in parse(result.gcode)
                                     for letter, _ in block.words))
                for program in (source, result.gcode):
                    self.assertEqual([number for block in parse(program)
                                      for letter, number in block.words if letter == "M"], [2.])
                self.assertTrue(all(m.feed and m.feed > 0 for m in output if m.kind != "RAPID"))
                if result.report["native_arcs"]["fillet_arc_count"]:
                    # A closure fillet may trim the nominal final vertex.
                    self.assertLess(min(distance_to_chord(output[-1].end, m.start, m.end)
                                        for m in raw if m.kind == "LINEAR"), 2e-5)
                else:
                    self.assertLess(math.dist(raw[-1].end, output[-1].end), 2e-5)
                for motion in output:
                    if motion.kind.startswith("ARC"):
                        cx, cy, radius, _, sweep = arc_geometry(motion)
                        self.assertTrue(all(math.isfinite(x) for x in (cx, cy, radius, sweep)))
                        self.assertLessEqual(abs(math.dist(motion.end[:2], (cx, cy))-radius), .005)
                if file.name.startswith(("01_", "02_", "03_", "04_")):
                    self.assertTrue(result.report["curve_reconstruction"]["applied"])
                    self.assertGreater(result.report["native_arcs"]["fitted_arc_count"], 0)
                if file.name.startswith("09_"):
                    self.assertTrue(result.report["curve_reconstruction"]["applied"])
                    self.assertGreater(result.report["native_arcs"]["fitted_arc_count"], 0)
                    self.assertTrue(all(row["arc_length_fraction"] > .9 for row in
                                        result.report["native_arcs"]["coverage_by_stroke"]))
                if file.name.startswith("06_") or file.name.startswith("08_"):
                    self.assertGreater(result.report["native_arcs"]["source_arc_count"], 0)
                if file.name.startswith(("13_", "14_")):
                    source_arcs = [m for m in raw if m.kind.startswith("ARC")]
                    exported_arcs = [m for m in output if m.kind.startswith("ARC")]
                    self.assertEqual(len(source_arcs), 3)
                    self.assertEqual(len(exported_arcs), 3)
                    for source_arc, exported_arc in zip(source_arcs, exported_arcs):
                        self.assertEqual(source_arc.kind, exported_arc.kind)
                        self.assertAlmostEqual(arc_geometry(source_arc)[-1],
                                               arc_geometry(exported_arc)[-1], places=7)
                if file.name.startswith("13_"):
                    self.assertEqual(result.report["native_arcs"]["source_arc_count"], 3)
                    self.assertTrue(all(abs(math.degrees(arc_geometry(m)[-1])-360) < 1e-7
                                        for m in output if m.kind == "ARC_CCW"))
                    arc_blocks = [b for b in parse(result.gcode)
                                  if any(letter == "G" and number == 3 for letter, number in b.words)]
                    self.assertEqual(len(arc_blocks), 3)
                    self.assertTrue(all(block.value("X") is None and block.value("Y") is None
                                        for block in arc_blocks))
                if file.name.startswith("14_"):
                    self.assertEqual(result.report["native_arcs"]["source_arc_count"], 3)
                    self.assertTrue(all(abs(math.degrees(arc_geometry(m)[-1])+180) < 1e-7
                                        for m in output if m.kind == "ARC_CW"))
                if file.name.startswith("06_"):
                    self.assertEqual(result.report["native_arcs"]["source_arc_count"], 12)
                    self.assertEqual(result.report["native_arcs"]["fillet_arc_count"], 0)
                    self.assertTrue(any(m.kind == "ARC_CW" for m in output))
                    self.assertTrue(any(m.kind == "ARC_CCW" for m in output))
                    source_arcs = [m for m in raw if m.kind.startswith("ARC")]
                    exported_arcs = [m for m in output if m.kind.startswith("ARC")]
                    self.assertEqual(len(source_arcs), len(exported_arcs))
                    for source_arc, exported_arc in zip(source_arcs, exported_arcs):
                        self.assertEqual(source_arc.kind, exported_arc.kind)
                        self.assertLess(math.dist(source_arc.start, exported_arc.start), 2e-5)
                        self.assertLess(math.dist(source_arc.end, exported_arc.end), 2e-5)
                if file.name.startswith("07_"):
                    self.assertEqual(len(result.report["native_arcs"]["coverage_by_stroke"]), 6)


if __name__ == "__main__":
    unittest.main()
