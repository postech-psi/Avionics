"""Exercise the real replay functions without importing the Qt/OpenGL UI."""
import ast
import csv
import math
from pathlib import Path
import re
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def replay_functions():
    tree = ast.parse((ROOT / 'ground-station/Groundstation.py').read_text(encoding='utf-8-sig'))
    names = {'_raw_health', '_to_float', 'read_log_frames', 'RAW_CSV_MAP',
             'FLOAT_KEYS', 'CSV_FIELDS', 'LEGACY_LINE'}
    nodes = [n for n in tree.body if
             isinstance(n, ast.FunctionDef) and n.name in names or
             isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in names for t in n.targets)]
    scope = {'csv': csv, 're': re, 'math': math}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), 'Groundstation.py', 'exec'), scope)
    return scope


class ReplayHealthTests(unittest.TestCase):
    def test_explicit_fault_notes_override_retained_sensor_values(self):
        decode = replay_functions()['_raw_health']
        retained = {'lat': 36., 'lon': 129., 'pressure': 1013.}
        cases = {
            'ok': 0xFF,
            'imu_dead|vz_off': 0x3A,
            'baro_dead|vz_off': 0x35,
            'imu_stale|vz_settle': 0x7E,
            'baro_stale|vz_settle': 0x7D,
            'gnss_dead': 0xEF,
            'no_baro_ref|vz_off': 0x1F,
            'vz_settle': 0x7F,
            'vz_off': 0x3F,
            'imu_dead|baro_dead|gnss_dead|no_baro_ref|vz_off': 0,
        }
        for note, expected in cases.items():
            with self.subTest(note=note):
                self.assertEqual(decode(retained, note), expected)

    def test_alive_receiver_without_fix_is_preserved(self):
        self.assertEqual(replay_functions()['_raw_health'](
            {'lat': 0., 'lon': 0., 'pressure': 1013.}, 'ok'), 0xFF)

    def test_raw_csv_faults_reach_replayed_frames(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'flight.csv'
            path.write_text('time_ms,state,lat,lon,pressure,note\n'
                            '1000,launch,36,129,1013,imu_dead|vz_off\n'
                            '1010,launch,36,129,1013,vz_settle\n', encoding='utf-8')
            frames = replay_functions()['read_log_frames'](path)
        self.assertEqual([f['health'] for f in frames], [0x3A, 0x7F])
        self.assertEqual([f['time'] for f in frames], [1., 1.01])

    def test_legacy_without_notes_keeps_previous_inference(self):
        self.assertEqual(replay_functions()['_raw_health'](
            {'lat': 36., 'lon': 129., 'pressure': 1013.}, ''), 0x3F)

    def test_ground_station_csv_keeps_recorded_health(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'ground.csv'
            path.write_text('wall,t,state,health\n12:00:00,1.5,LAUNCH,58\n', encoding='utf-8')
            frames = replay_functions()['read_log_frames'](path)
        self.assertEqual(frames[0]['health'], 0x3A)


if __name__ == '__main__':
    unittest.main()
