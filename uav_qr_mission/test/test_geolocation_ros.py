"""ROS message validation and synchronization checks; no ROS master required."""
import math
import unittest
from unittest.mock import patch

import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from test_geolocation import config
from uav_qr_mission.geolocation import WGS84_A, geodetic_to_ecef
from uav_qr_mission.geolocation_ros import Geolocation


class GeolocationInputsTest(unittest.TestCase):
    def setUp(self):
        cfg = config()
        cfg['publish_static_tf'] = False
        self.geo = Geolocation(cfg)
        self.report = TransformStamped()
        self.report.header.frame_id = self.geo.installation.camera
        self.report.header.stamp = rospy.Time.from_sec(100)
        self.report.transform.translation.z = 2
        self.fix = NavSatFix()
        self.fix.header.stamp = rospy.Time.from_sec(100)
        self.fix.status.status = NavSatStatus.STATUS_FIX
        self.fix.latitude, self.fix.longitude, self.fix.altitude = 0, 0, 100
        self.imu = Imu()
        self.imu.header.stamp = rospy.Time.from_sec(100)
        self.imu.orientation.w = 1

    def status(self):
        return self.geo.locate(self.report)[1]

    def test_missing_inputs_and_success_without_arena_tf(self):
        self.assertEqual(self.status(), 'WAIT_FIX')
        self.geo.fixes.add(self.fix)
        self.assertEqual(self.status(), 'WAIT_ATTITUDE')
        self.geo.attitudes.add(self.imu)
        (position, fix_stamp, imu_stamp), status = self.geo.locate(self.report)
        self.assertEqual(status, 'VALID')
        self.assertEqual(position.header.stamp, self.report.header.stamp)
        self.assertEqual(fix_stamp, self.fix.header.stamp)
        self.assertEqual(imu_stamp, self.imu.header.stamp)
        self.assertEqual(position.position_covariance_type, NavSatFix.COVARIANCE_TYPE_UNKNOWN)
        xyz = geodetic_to_ecef(position.latitude, position.longitude, position.altitude)
        for a, b in zip(xyz, [WGS84_A+97.725, -0.105, -0.1585]):
            self.assertAlmostEqual(a, b, places=6)

    def test_stale_and_future_inputs_are_rejected(self):
        for offset in (-0.21, 0.21):
            self.fix.header.stamp = rospy.Time.from_sec(100+offset)
            self.geo.fixes.add(self.fix)
            self.assertEqual(self.status(), 'FIX_TIME_MISMATCH')
        self.fix.header.stamp = rospy.Time.from_sec(100)
        self.geo.fixes.add(self.fix)
        for offset in (-0.11, 0.11):
            self.imu.header.stamp = rospy.Time.from_sec(100+offset)
            self.geo.attitudes.add(self.imu)
            self.assertEqual(self.status(), 'ATTITUDE_TIME_MISMATCH')

    def test_time_tolerance_boundary_at_unix_epoch_scale(self):
        self.report.header.stamp = rospy.Time(1794748000)
        self.fix.header.stamp = self.report.header.stamp + rospy.Duration(0.2)
        self.imu.header.stamp = self.report.header.stamp - rospy.Duration(0.1)
        self.geo.fixes.add(self.fix)
        self.geo.attitudes.add(self.imu)
        self.assertEqual(self.status(), 'VALID')

    def test_no_fix_and_invalid_quaternion(self):
        self.fix.status.status = NavSatStatus.STATUS_NO_FIX
        self.geo.fixes.add(self.fix)
        self.assertEqual(self.status(), 'FIX_INVALID')
        self.fix.status.status = NavSatStatus.STATUS_FIX
        self.geo.attitudes.add(self.imu)
        self.imu.orientation.w = 0
        self.assertEqual(self.status(), 'ATTITUDE_INVALID')
        self.imu.orientation.w = math.nan
        self.assertEqual(self.status(), 'ATTITUDE_INVALID')
        self.imu.orientation.w = 1
        self.imu.orientation_covariance[0] = -1
        self.assertEqual(self.status(), 'ATTITUDE_INVALID')

    def test_invalid_geodetic_and_camera_frame(self):
        self.geo.fixes.add(self.fix)
        self.geo.attitudes.add(self.imu)
        for latitude, altitude in ((91, 100), (0, math.nan)):
            self.fix.latitude, self.fix.altitude = latitude, altitude
            self.assertEqual(self.status(), 'FIX_INVALID')
        self.report.header.frame_id = 'unknown_camera'
        self.assertEqual(self.status(), 'CAMERA_FRAME_MISMATCH')

    def test_select_nearest_not_last_received(self):
        import copy
        self.geo.fixes.add(copy.deepcopy(self.fix))
        self.fix.header.stamp = rospy.Time.from_sec(99)
        self.geo.fixes.add(self.fix)
        self.geo.attitudes.add(self.imu)
        self.assertEqual(self.status(), 'VALID')

    def test_expire_cache_and_ignore_zero_stamp(self):
        with patch('uav_qr_mission.geolocation_ros.time.monotonic', return_value=10):
            self.geo.fixes.add(self.fix)
        with patch('uav_qr_mission.geolocation_ros.time.monotonic', return_value=16):
            self.assertEqual(self.status(), 'WAIT_FIX')
        self.fix.header.stamp = rospy.Time(0)
        self.geo.fixes.add(self.fix)
        self.assertEqual(self.status(), 'WAIT_FIX')


if __name__ == '__main__':
    unittest.main()
