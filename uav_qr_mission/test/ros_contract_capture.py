#!/usr/bin/env python3
"""Stationary capture contract. Requires an isolated loopback mission node only."""
import os
import threading
import time
import unittest
from urllib.parse import urlparse

import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, NavSatFix
from uav_odin_qr.msg import Detection, DetectionArray, QrTransform, QrTransformArray
from uav_qr_mission.msg import MissionResult
from uav_qr_mission.srv import SetTask, CaptureQr, CaptureQrRequest, FinishSurvey


class CaptureContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        uri = urlparse(os.environ.get('ROS_MASTER_URI', ''))
        if uri.hostname not in ('127.0.0.1', 'localhost') or uri.port == 11311:
            raise RuntimeError('Tests must not connect to the aircraft ROS master')
        rospy.init_node('capture_contract', anonymous=True)
        cls.latest = None
        cls.condition = threading.Condition()
        def result_cb(message):
            with cls.condition:
                cls.latest = message
                cls.condition.notify_all()
        cls.sub = rospy.Subscriber('/uav_qr_mission/result', MissionResult, result_cb)
        cls.detections = rospy.Publisher('/uav_odin_qr/detections', DetectionArray, queue_size=10)
        cls.targets = rospy.Publisher('/uav_odin_qr/tf_targets', QrTransformArray, queue_size=10)
        cls.fix = rospy.Publisher('/mavros/global_position/raw/fix', NavSatFix, queue_size=10)
        cls.imu = rospy.Publisher('/mavros/imu/data', Imu, queue_size=10)
        rospy.wait_for_service('/uav_qr_mission/capture', timeout=10)
        cls.task = rospy.ServiceProxy('/uav_qr_mission/set_task', SetTask)
        cls.capture = rospy.ServiceProxy('/uav_qr_mission/capture', CaptureQr)
        cls.finish = rospy.ServiceProxy('/uav_qr_mission/finish_survey', FinishSurvey)
        deadline = time.monotonic()+5
        while not all(p.get_num_connections() for p in (cls.detections,cls.targets,cls.fix,cls.imu)):
            if time.monotonic() > deadline: raise RuntimeError('No subscribers')
            time.sleep(.02)

    def wait(self, predicate, timeout=3):
        with self.condition:
            deadline = time.monotonic()+timeout
            while self.latest is None or not predicate(self.latest):
                remaining = deadline-time.monotonic()
                if remaining <= 0: self.fail('Capture result timed out: {}'.format(self.latest))
                self.condition.wait(remaining)
            return self.latest

    def setUp(self):
        self.mission = 'capture_'+str(time.monotonic_ns())
        self.assertTrue(self.task(self.mission,['A','B','C'],'C',True).accepted)
        self.wait(lambda r:r.mission_id == self.mission)

    def frame(self, stamp=None, depth_stamp=None):
        stamp = stamp or rospy.Time.now()
        depth_stamp = depth_stamp or stamp
        fix = NavSatFix()
        fix.header.stamp = depth_stamp
        fix.latitude,fix.longitude,fix.altitude = 0,0,100
        imu = Imu()
        imu.header.stamp = depth_stamp
        imu.orientation.w = 1
        self.fix.publish(fix)
        self.imu.publish(imu)
        detections,targets = DetectionArray(),QrTransformArray()
        detections.header.stamp = targets.header.stamp = stamp
        for i,content in enumerate(['A','B','C']):
            detections.detections.append(Detection(content=content,center_u=i,center_v=0,depth_valid=True))
            transform = TransformStamped()
            transform.header.stamp = depth_stamp
            transform.header.frame_id = 'uav_camera_optical_frame'
            transform.child_frame_id = 'test_'+content
            transform.transform.translation.z = 2
            transform.transform.rotation.w = 1
            targets.targets.append(QrTransform(content=content,center_u=i,center_v=0,transform=transform))
        self.targets.publish(targets)
        self.detections.publish(detections)

    def request(self, capture_id='stop', timeout=2, command=CaptureQrRequest.START, after=None):
        return self.capture(command,self.mission,capture_id,after or rospy.Time.now(),timeout)

    def test_fresh_capture_localizes_all_codes_and_skips_old_depth(self):
        old = rospy.Time.now()-rospy.Duration(1)
        self.frame(old)
        time.sleep(.1)
        self.assertFalse(any(self.latest.found))
        after = rospy.Time.now()
        self.assertTrue(self.request(after=after).accepted)
        self.frame(depth_stamp=old)  # fresh RGB with cached in-flight depth is not a capture
        time.sleep(.1)
        self.assertFalse(self.latest.capture_done)
        self.frame()
        result = self.wait(lambda r:r.capture_done)
        self.assertTrue(result.capture_success)
        self.assertTrue(result.all_localized)
        self.assertEqual([t.content for t in result.targets],['A','B','C'])
        self.assertTrue(all(t.position.header.stamp >= after for t in result.targets))
        self.assertTrue(all(t.revision == 1 for t in result.targets))
        self.assertTrue(self.request(after=after).accepted)  # retry does not recapture
        self.assertTrue(self.finish(self.mission).result.survey_finished)
        self.assertFalse(self.request('late_stop').accepted)

    def test_cancel_before_start_and_wrong_mission(self):
        self.assertTrue(self.request('cancelled',command=CaptureQrRequest.CANCEL).accepted)
        self.assertFalse(self.request('cancelled').accepted)
        self.assertFalse(self.capture(0,'another','x',rospy.Time.now(),1).accepted)

    def test_timeout_and_cancel_ignore_later_frames(self):
        self.assertTrue(self.request('no_image',timeout=.15).accepted)
        self.wait(lambda r:r.capture_done and r.capture_status == 'TIMEOUT_WAIT_FRAME')
        self.frame()
        time.sleep(.1)
        self.assertFalse(any(self.latest.found))
        self.assertTrue(self.request('cancel_now').accepted)
        self.assertTrue(self.request('cancel_now',command=CaptureQrRequest.CANCEL).accepted)
        self.frame()
        self.wait(lambda r:r.capture_status == 'CANCELLED')
        self.assertFalse(any(self.latest.found))


if __name__ == '__main__':
    unittest.main()
