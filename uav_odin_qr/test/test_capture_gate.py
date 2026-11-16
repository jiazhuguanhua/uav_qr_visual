import threading
import unittest

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Time
from uav_odin_qr.node import OdinQr


class CaptureGateTest(unittest.TestCase):
    def setUp(self):
        # Exercise the actual ingress gate without starting ROS, OYO or hardware.
        self.node = OdinQr.__new__(OdinQr)
        self.node.condition = threading.Condition()
        self.node.triggered = False
        self.node.capture_after = 0
        self.node.pending = None

    def image(self, stamp):
        image = Image()
        image.header.stamp = rospy.Time(stamp)
        return image

    def test_flight_pauses_decode_and_one_trigger_accepts_only_one_fresh_frame(self):
        self.node.on_image(self.image(1))
        self.assertIsNotNone(self.node.pending)
        self.node.on_mode(Bool(False))
        self.node.on_image(self.image(2))
        self.assertIsNone(self.node.pending)
        self.node.on_capture(Time(rospy.Time(10)))
        self.node.on_image(self.image(9))
        self.assertIsNone(self.node.pending)
        image = self.image(10)
        self.node.on_image(image)
        self.node.on_image(self.image(11))
        self.assertIs(self.node.pending[0], image)

    def test_cancel_and_resume_continuous_mode(self):
        self.node.on_capture(Time(rospy.Time(10)))
        self.node.on_capture(Time(rospy.Time(0)))
        self.node.on_image(self.image(11))
        self.assertIsNone(self.node.pending)
        self.node.on_mode(Bool(True))
        self.node.on_image(self.image(12))
        self.assertEqual(self.node.pending[0].header.stamp, rospy.Time(12))


if __name__ == '__main__':
    unittest.main()
