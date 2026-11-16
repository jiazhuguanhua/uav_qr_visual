#!/usr/bin/env python3
"""Run against uav_qr_mission on an isolated loopback ROS master only."""
import os
import threading
import time
import unittest
from urllib.parse import urlparse

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage
from uav_odin_qr.msg import Detection, DetectionArray, QrTransform, QrTransformArray
from uav_qr_mission.msg import MissionResult
from uav_qr_mission.srv import SetTask


class Wgs84Contract(unittest.TestCase):
    sequence = 0

    @classmethod
    def setUpClass(cls):
        uri = urlparse(os.environ.get('ROS_MASTER_URI', ''))
        if uri.hostname not in ('127.0.0.1', 'localhost', '::1'):
            raise RuntimeError('Use an isolated loopback ROS master, never the vehicle master')
        rospy.init_node('wgs84_contract', anonymous=True)
        cls.condition = threading.Condition()
        cls.latest = None
        cls.static_edges = {}

        def result_cb(message):
            with cls.condition:
                cls.latest = message
                cls.condition.notify_all()

        def static_cb(message):
            with cls.condition:
                for transform in message.transforms:
                    cls.static_edges[transform.child_frame_id] = transform
                cls.condition.notify_all()

        cls.subs = [rospy.Subscriber('/uav_qr_mission/result', MissionResult, result_cb),
                    rospy.Subscriber('/tf_static', TFMessage, static_cb)]
        cls.detections = rospy.Publisher('/uav_odin_qr/detections', DetectionArray, queue_size=10)
        cls.targets = rospy.Publisher('/uav_odin_qr/tf_targets', QrTransformArray, queue_size=10)
        cls.fixes = rospy.Publisher('/mavros/global_position/raw/fix', NavSatFix, queue_size=10)
        cls.attitudes = rospy.Publisher('/mavros/imu/data', Imu, queue_size=10)
        rospy.wait_for_service('/uav_qr_mission/set_task', timeout=15)
        cls.set_task = rospy.ServiceProxy('/uav_qr_mission/set_task', SetTask)
        deadline = time.monotonic() + 10
        while not all(p.get_num_connections() for p in
                      (cls.detections, cls.targets, cls.fixes, cls.attitudes)):
            if time.monotonic() > deadline:
                raise RuntimeError('Mission subscriptions did not connect')
            time.sleep(0.02)

    def wait(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        with self.condition:
            while not predicate(self.__class__.latest):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.fail('Result timeout; latest={}'.format(self.__class__.latest))
                self.condition.wait(remaining)
            return self.__class__.latest

    def setUp(self):
        type(self).sequence += 1
        self.stamp = rospy.Time(1000 + self.sequence * 10)
        self.mission_id = 'contract_{}'.format(self.sequence)
        response = self.set_task(self.mission_id, ['A', 'B', 'C'], 'A', False)
        self.assertTrue(response.accepted)
        self.wait(lambda r: r is not None and r.mission_id == self.mission_id)

    def telemetry(self, fix_stamp=None, imu_stamp=None):
        fix = NavSatFix()
        fix.header.stamp = fix_stamp or self.stamp
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude, fix.longitude, fix.altitude = 0, 0, 100
        imu = Imu()
        imu.header.stamp = imu_stamp or self.stamp
        imu.orientation.w = 1
        self.fixes.publish(fix)
        self.attitudes.publish(imu)

    def observe(self, contents=('A', 'B', 'C'), camera='uav_camera_optical_frame'):
        detections = DetectionArray()
        detections.header.stamp = self.stamp
        detections.detections = [Detection(content=c, center_u=100+i*10, center_v=80,
                                           depth_valid=True) for i, c in enumerate(contents)]
        targets = QrTransformArray()
        targets.header = detections.header
        for d in detections.detections:
            t = TransformStamped()
            t.header.stamp = self.stamp
            t.header.frame_id = camera
            t.child_frame_id = 'synthetic_qr'
            t.transform.translation.z = 2
            t.transform.rotation.w = 1
            targets.targets.append(QrTransform(content=d.content, center_u=d.center_u,
                                               center_v=d.center_v, transform=t))
        self.targets.publish(targets)  # Also exercise reverse topic arrival order.
        self.detections.publish(detections)

    def test_camera_mismatch(self):
        self.telemetry()
        self.observe(camera='different_optical_frame')
        result = self.wait(lambda r: r.wgs84_status == 'CAMERA_FRAME_MISMATCH')
        self.assertFalse(result.wgs84_position_valid)

    def test_delayed_telemetry_is_retried(self):
        self.observe()
        self.wait(lambda r: r.report_tf_valid)
        self.assertFalse(self.latest.wgs84_position_valid)
        self.telemetry()
        result = self.wait(lambda r: r.wgs84_position_valid)
        self.assertEqual(result.wgs84_position.header.stamp, self.stamp)
        self.assertEqual(result.wgs84_fix_stamp, self.stamp)
        self.assertEqual(result.wgs84_attitude_stamp, self.stamp)

    def test_duplicate_selected_content_is_not_localized(self):
        self.telemetry()
        self.observe(contents=('A', 'A', 'B'))
        result = self.wait(lambda r: list(r.found) == [True, True, False])
        self.assertFalse(result.report_tf_valid)
        self.assertFalse(result.wgs84_position_valid)

    def test_expired_observation_does_not_accept_late_fix(self):
        self.observe()
        self.wait(lambda r: r.wgs84_status == 'OBSERVATION_EXPIRED')
        self.telemetry()
        time.sleep(0.15)
        self.assertFalse(self.latest.wgs84_position_valid)
        self.stamp += rospy.Duration(1)
        self.telemetry()
        self.observe()
        self.wait(lambda r: r.wgs84_position_valid)

    def test_save_hold_and_new_task_reset(self):
        self.telemetry()
        self.observe()
        result = self.wait(lambda r: r.wgs84_position_valid)
        self.assertAlmostEqual(result.wgs84_position.altitude, 97.725, places=5)
        self.assertEqual(result.wgs84_position.header.frame_id, 'wgs84')
        self.assertFalse(result.position_valid)  # WGS84 does not depend on arena TF.
        self.assertFalse(result.complete)       # Legacy completion still requires arena.
        self.assertTrue(all(result.found))
        self.assertEqual(self.set_task(self.mission_id, ['A', 'B', 'C'], 'A', False).status, 'ALREADY_ACTIVE')
        self.stamp += rospy.Duration(1)
        self.observe(contents=())
        result = self.wait(lambda r: not r.report_tf_valid)
        self.assertTrue(result.wgs84_position_valid)
        self.assertEqual(result.wgs84_status, 'VALID')
        self.set_task(self.mission_id+'_new', ['A', 'B', 'C'], 'A', False)
        result = self.wait(lambda r: r.mission_id.endswith('_new'))
        self.assertFalse(result.wgs84_position_valid)
        self.assertEqual(result.wgs84_status, 'WAIT_OBSERVATION')

    def test_stale_samples_rejected_then_recovery(self):
        self.telemetry(self.stamp-rospy.Duration(1), self.stamp-rospy.Duration(1))
        self.observe()
        result = self.wait(lambda r: r.wgs84_status == 'FIX_TIME_MISMATCH')
        self.assertFalse(result.wgs84_position_valid)
        self.telemetry()
        self.wait(lambda r: r.wgs84_position_valid)

    def test_static_installation_frames(self):
        deadline = time.monotonic() + 3
        with self.condition:
            while 'uav_rtk_ant2' not in self.static_edges:
                remaining = deadline-time.monotonic()
                self.assertGreater(remaining, 0)
                self.condition.wait(remaining)
        self.assertEqual(self.static_edges['uav_camera'].header.frame_id, 'uav_base_link')
        self.assertEqual(self.static_edges['uav_camera_optical_frame'].header.frame_id, 'uav_camera')
        self.assertAlmostEqual(self.static_edges['uav_rtk_ant1'].transform.translation.y, 0.1585)

    def test_z_arena_and_legacy_completion_still_work(self):
        self.arena_broadcaster = tf2_ros.StaticTransformBroadcaster()
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'arena'
        transform.child_frame_id = 'uav_base_link'
        transform.transform.rotation.w = 1
        self.arena_broadcaster.sendTransform(transform)
        time.sleep(0.2)
        self.telemetry()
        self.observe()
        result = self.wait(lambda r: r.complete and r.wgs84_position_valid)
        self.assertEqual(result.position.header.frame_id, 'arena')
        self.assertAlmostEqual(result.position.point.x, -0.105)
        self.assertAlmostEqual(result.position.point.y, 0.0)
        self.assertAlmostEqual(result.position.point.z, -2.17)


if __name__ == '__main__':
    unittest.main()
