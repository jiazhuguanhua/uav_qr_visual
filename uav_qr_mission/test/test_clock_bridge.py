import unittest
from unittest.mock import patch

import rospy
from test_geolocation_ros import GeolocationInputsTest
from uav_qr_mission.clock_bridge import ClockBridge


class ClockBridgeTest(unittest.TestCase):
    def bridge(self, skew):
        bridge = ClockBridge()
        bridge.update_sync(1794820000000000000, 2)
        for boot in (5000000, 5001000, 5002000):
            bridge.update_system(1794820000000000+boot*1000+int(skew*1e6), boot)
        return bridge

    def test_both_directions_and_integer_precision(self):
        for skew in (-.8, .5, .8, 1.58):
            bridge = self.bridge(skew)
            self.assertEqual(bridge.offset(), (-int(skew*1e9), 'VALID'))

    def test_missing_stale_and_jump_reconverge(self):
        with patch('uav_qr_mission.clock_bridge.time.monotonic', return_value=10):
            bridge = self.bridge(.8)
        with patch('uav_qr_mission.clock_bridge.time.monotonic', return_value=16):
            self.assertEqual(bridge.offset()[1], 'WAIT_CLOCK_SYNC')
            bridge.update_sync(1794820000000000000, 1)
            self.assertEqual(bridge.offset()[1], 'WAIT_CLOCK_ANCHOR')
        bridge = self.bridge(.8)
        bridge.update_sync(1794820001000000000, 1)
        self.assertEqual(bridge.offset()[1], 'WAIT_CLOCK_ANCHOR')
        self.assertEqual(ClockBridge().offset()[1], 'WAIT_CLOCK_SYNC')

    def test_high_rtt_and_nonfinite_rejected(self):
        bridge = ClockBridge()
        for rtt in (11, -1, float('nan')):
            bridge.update_sync(123, rtt)
        self.assertIsNone(bridge.sync)

    def test_geolocation_matches_physical_time_preserves_raw_stamp(self):
        fixture = GeolocationInputsTest()
        fixture.setUp()
        geo = fixture.geo
        geo.clock_bridge = self.bridge(.8)
        fixture.report.header.stamp = rospy.Time(1794825000)
        fixture.fix.header.stamp = rospy.Time(1794825000, 800000000)
        fixture.imu.header.stamp = fixture.report.header.stamp
        geo.fixes.add(fixture.fix)
        geo.attitudes.add(fixture.imu)
        result, status = geo.locate(fixture.report)
        self.assertEqual(status, 'VALID')
        self.assertEqual(result[1], fixture.fix.header.stamp)
        # A genuine measurement skew remains invalid after removing clock offset.
        fixture.report.header.stamp += rospy.Duration(.3)
        self.assertEqual(geo.locate(fixture.report)[1], 'FIX_TIME_MISMATCH')


if __name__ == '__main__':
    unittest.main()
