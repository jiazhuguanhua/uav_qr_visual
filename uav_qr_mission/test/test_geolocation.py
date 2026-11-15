import copy
import math
from pathlib import Path
import unittest

import yaml

from uav_qr_mission.geolocation import (
    Installation, WGS84_A, WGS84_F, ecef_to_geodetic, geodetic_to_ecef,
    offset_wgs84, quaternion_from_rpy,
)


def config():
    path = Path(__file__).resolve().parents[1] / 'config' / 'geolocation.yaml'
    return yaml.safe_load(path.read_text())['geolocation']


class GeolocationMathTest(unittest.TestCase):
    def assertVector(self, actual, expected, places=8):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=places)

    def test_camera_axes_and_ant1_lever_arm(self):
        installation = Installation(config())
        # optical x -> -body y; optical y -> -body x; optical z -> -body z.
        self.assertVector(installation.antenna_to_target_enu([1, 2, 3], [0, 0, 0, 1]),
                          [-2.105, -1.1585, -3.275])

    def test_yaw_and_roll_rotate_the_lever_arm_too(self):
        installation = Installation(config())
        self.assertVector(installation.antenna_to_target_enu(
            [0, 0, 2], quaternion_from_rpy([0, 0, math.pi/2])), [0.1585, -0.105, -2.275])
        self.assertVector(installation.antenna_to_target_enu(
            [0, 0, 2], quaternion_from_rpy([math.pi/2, 0, 0])), [-0.105, 2.275, -0.1585])

    def test_configured_antenna_is_not_hardcoded(self):
        cfg = config()
        cfg['rtk_frame'] = 'uav_rtk_ant2'
        self.assertVector(Installation(cfg).antenna_to_target_enu([0, 0, 2], [0, 0, 0, 1]),
                          [-0.105, 0.1585, -2.275])

    def test_nested_translation_and_unordered_config(self):
        cfg = config()
        cfg['static_transforms'][1]['xyz'] = [0.2, 0.3, 0.4]
        cfg['static_transforms'].reverse()
        self.assertVector(Installation(cfg).camera_xyz, [0.295, 0.3, -0.37])

    def test_invalid_installations(self):
        original = config()
        mutations = []
        duplicate = copy.deepcopy(original)
        duplicate['static_transforms'].append(copy.deepcopy(duplicate['static_transforms'][0]))
        mutations.append(duplicate)
        for parent in ('uav_camera_optical_frame', 'missing'):
            cfg = copy.deepcopy(original)
            cfg['static_transforms'][0]['parent'] = parent
            mutations.append(cfg)
        invalid = copy.deepcopy(original)
        invalid['static_transforms'][0]['xyz'][0] = math.nan
        mutations.append(invalid)
        for cfg in mutations:
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                Installation(cfg)

    def test_wgs84_reference_points(self):
        self.assertVector(geodetic_to_ecef(0, 0, 0), [WGS84_A, 0, 0])
        self.assertVector(geodetic_to_ecef(0, 90, 0), [0, WGS84_A, 0])
        self.assertVector(geodetic_to_ecef(90, 0, 0), [0, 0, WGS84_A*(1-WGS84_F)])

    def test_roundtrip_poles_dateline_and_heights(self):
        for lat in (-90, -89.999, -45, 0, 31.2, 89.999, 90):
            for lon in (-180, -123, 0, 121.5, 180):
                for altitude in (-100, 0, 3000):
                    with self.subTest(lla=(lat, lon, altitude)):
                        result = ecef_to_geodetic(*geodetic_to_ecef(lat, lon, altitude))
                        self.assertAlmostEqual(result[0], lat, places=9)
                        if abs(lat) < 90:
                            self.assertAlmostEqual(result[1], lon, places=9)
                        self.assertAlmostEqual(result[2], altitude, places=6)

    def test_enu_basis_at_equator_and_eastern_hemisphere(self):
        for lat, lon, expected in (
                (0, 0, [WGS84_A+103, 1, 2]),
                (0, 90, [-1, WGS84_A+103, 2])):
            result = offset_wgs84(lat, lon, 100, [1, 2, 3])
            self.assertVector(geodetic_to_ecef(*result), expected, places=6)

    def test_invalid_inputs(self):
        for lla in ([91, 0, 0], [0, 181, 0], [0, 0, math.nan]):
            with self.assertRaises(ValueError):
                offset_wgs84(*lla, [0, 0, 1])
        with self.assertRaises(ValueError):
            Installation(config()).antenna_to_target_enu([0, 0, 2], [0, 0, 0, 0])


if __name__ == '__main__':
    unittest.main()
