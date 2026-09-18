import importlib.util
import tempfile
import unittest
import subprocess
import sys
from pathlib import Path

spec = importlib.util.spec_from_file_location('thrust_import', Path(__file__).resolve().parents[1] / 'tools/import_thrust_data.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ThrustImportTests(unittest.TestCase):
    def read(self, text):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'bench.csv'
            path.write_text(text)
            return module.read_measurements(path)

    def test_newtons_sorted_with_optional_torque(self):
        data = self.read('motor_command_0_1,thrust_N,torque_Nm\n1,13.5,0.18\n0,0,0\n0.5,4,0.06\n')
        self.assertEqual([p['command'] for p in data['points']], [0, .5, 1])
        self.assertEqual(data['points'][1]['torque_Nm'], .06)

    def test_grams_are_converted_to_force(self):
        data = self.read('motor_command_0_1,thrust_g\n0,0\n1,1000\n')
        self.assertAlmostEqual(data['points'][-1]['thrust_N'], 9.80665)
        self.assertNotIn('torque_Nm', data['points'][0])

    def test_no_extrapolation_or_nonphysical_curves(self):
        for rows in ['0.2,2\n1,10', '0,0\n0.8,10', '0,0\n1,nan', '0,0\n1,inf',
                     '0,0\n0.5,7\n1,6', '0,0\n0.5,4\n0.5,4\n1,10', '0,0\n1,-5']:
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                self.read('motor_command_0_1,thrust_N\n' + rows + '\n')

    def test_ambiguous_units_and_missing_torque_rejected(self):
        for data in ['motor_command_0_1,thrust_N,thrust_g\n0,0,0\n1,10,1000\n',
                     'motor_command_0_1,thrust_N,torque_Nm\n0,0,0\n1,10,\n']:
            with self.subTest(data=data), self.assertRaises(ValueError):
                self.read(data)

    def test_cli_protects_existing_measurements(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'bench.csv'
            source.write_text('motor_command_0_1,thrust_N\n0,0\n1,10\n')
            output = Path(folder) / 'curve.json'
            command = [sys.executable, module.__file__, str(source), '--output', str(output)]
            first = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(first.returncode, 0, first.stderr)
            original = output.read_bytes()
            source.write_text('motor_command_0_1,thrust_N\n0,0\n1,12\n')
            second = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(second.returncode, 2)
            self.assertEqual(output.read_bytes(), original)
            third = subprocess.run(command + ['--force'], capture_output=True, text=True)
            self.assertEqual(third.returncode, 0, third.stderr)
            self.assertNotEqual(output.read_bytes(), original)

    def test_models_have_independent_measurement_destinations(self):
        self.assertNotEqual(module.MODEL_OUTPUTS['gopro_drone'], module.MODEL_OUTPUTS['dji_fpv'])
        self.assertEqual(module.MODEL_OUTPUTS['gopro_drone'], module.DEFAULT_OUTPUT)


if __name__ == '__main__':
    unittest.main()
