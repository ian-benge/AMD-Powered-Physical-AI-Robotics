import json
from pathlib import Path
import tempfile
import unittest

from rover_navigation.platform_telemetry import read_temperature, write_temperature


class TemperatureTests(unittest.TestCase):
    def test_fresh_native_reading_is_displayed_but_never_used_as_motion_authority(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'temperature.json'
            write_temperature(path, dict(utc='2026-11-10T02:30:00Z', tctl_c=81.5), 'boot-one', '/case-one')
            stamp = json.loads(path.read_text())['wall_monotonic']
            value = read_temperature(path, 'boot-one', stamp+.5)
            self.assertEqual(value['tctl_c'], 81.5)
            self.assertEqual(value['age_seconds'], .5)
            self.assertEqual((value['qualification_stop_c'], value['native_stop_c']), (87, 90))
            self.assertNotIn('ready', value)
            self.assertIsNone(read_temperature(path, 'boot-two', stamp+.5))
            self.assertIsNone(read_temperature(path, 'boot-one', stamp+2.01))
            self.assertIsNone(read_temperature(path, 'boot-one', stamp-.06))
            self.assertEqual(list(Path(folder).glob('.demo-temperature-*')), [])

    def test_partial_corrupt_and_nonphysical_readings_remain_unavailable(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'temperature.json'
            self.assertIsNone(read_temperature(path, 'boot', 100.))
            for bad in ('{', '[]', 'null'):
                path.write_text(bad)
                self.assertIsNone(read_temperature(path, 'boot', 100.))
            good = dict(source='native_capture_guard', boot_id='boot', utc='utc',
                        wall_monotonic=100., tctl_c=80.)
            for changes in (dict(tctl_c=float('nan')), dict(tctl_c=True), dict(tctl_c=126),
                            dict(wall_monotonic='100'), dict(source='unverified')):
                path.write_text(json.dumps(dict(good, **changes)))
                self.assertIsNone(read_temperature(path, 'boot', 100.))


if __name__ == '__main__':
    unittest.main()
