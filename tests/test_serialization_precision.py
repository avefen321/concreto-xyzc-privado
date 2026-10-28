from pathlib import Path
import re
import unittest

from backend.core.config import Config
from backend.core.conversion import convert


ROOT = Path(__file__).resolve().parent.parent
WORDS = re.compile(r'\b([XYZCF])([-+]?\d+\.\d+)')


class SerializationPrecisionTests(unittest.TestCase):
    def test_export_uses_exactly_the_selected_decimals(self):
        source = (ROOT / 'examples' / 'cuadrado_300_6_capas.ngc').read_text()
        for decimals, expected in ((3, 'X277.765 Y1.022'),
                                   (4, 'X277.7646 Y1.0222')):
            with self.subTest(decimals=decimals):
                # Coordinates below belong to the historical 30 mm nozzle profile.
                config = Config.from_dict({'nozzle': {'length_mm': 30}, 'native_arcs': {'enabled': False}, 'serialization': {'xyz_decimals': decimals,
                                                              'c_decimals': decimals,
                                                              'feed_decimals': decimals}})
                result = convert(source, config)
                self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
                self.assertIn(expected, result.gcode)
                for line in result.gcode.splitlines():
                    if not line.startswith(('G0 ', 'G1 ')):
                        continue
                    for axis, value in WORDS.findall(line):
                        self.assertEqual(len(value.split('.')[1]), decimals,
                                         (axis, value, line))
                audit = result.report['validators']['Serialization']
                self.assertEqual(audit['status'], 'PASS')
                self.assertLessEqual(audit['max_xyz_rounding_error_mm'],
                                     audit['xyz_rounding_budget_mm'])
                self.assertGreater(audit['max_xyz_rounding_error_mm'],
                                   config.geometry.max_xy_deviation_mm)

    def test_feed_rounds_without_adding_extra_digits(self):
        source = 'G21 G90\nG1 X10.12345 Y0 F2400.12345\n'
        config = Config.from_dict({'geometry': {'mode': 'PRESERVE_PATH'},
                                   'serialization': {'xyz_decimals': 3, 'c_decimals': 3,
                                                     'feed_decimals': 1}})
        result = convert(source, config)
        self.assertNotEqual(result.report['status'], 'FAIL', result.report['errors'])
        self.assertIn('X10.123 Y0.000', result.gcode)
        self.assertIn('F2400.1', result.gcode)
        self.assertNotIn('F2400.123', result.gcode)


if __name__ == '__main__':
    unittest.main()
