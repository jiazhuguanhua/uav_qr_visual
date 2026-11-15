"""ROS image subscriber and per-frame ODIN QR publisher."""

import math
import threading
import time

import cv2
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from uav_oyo.srv import QueryPixelResponse

from .decoder import decode_centers
from .msg import Detection, DetectionArray, QrTransform, QrTransformArray
from .oyo_client import OyoClient
from .qr_tracks import QrTracks


class OdinQr:
    def __init__(self):
        image_topic = rospy.get_param('~image_topic', '/odin1/image/undistorted')
        query_service = rospy.resolve_name(rospy.get_param('~query_service', '/uav_oyo/query_pixel'))
        self.camera_frame = rospy.get_param('~camera_frame', '').strip().lstrip('/')
        self.query_timeout = float(rospy.get_param('~query_timeout_s', 0.8))
        if not math.isfinite(self.query_timeout) or not 0 < self.query_timeout <= 0.8:
            raise ValueError('~query_timeout_s must be greater than zero and at most 0.8')
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.client = OyoClient(query_service, rospy.get_name())
        self.publisher = rospy.Publisher('~detections', DetectionArray, queue_size=1)
        self.tf_targets = rospy.Publisher('~tf_targets', QrTransformArray, queue_size=1)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.tracks = QrTracks()
        self.last_frame = ''
        self.last_stamp = None
        self.condition = threading.Condition()
        self.pending = None
        self.stopped = False
        self.subscriber = rospy.Subscriber(
            image_topic, Image, self.on_image, queue_size=1, buff_size=16 * 1024 * 1024
        )
        rospy.on_shutdown(self.stop)
        rospy.loginfo('ODIN QR input %s, OYO %s, output %s',
                      image_topic, query_service, self.publisher.resolved_name)

    def on_image(self, image):
        with self.condition:
            self.pending = (image, time.monotonic())
            self.condition.notify()

    def stop(self):
        with self.condition:
            self.stopped = True
            self.condition.notify()

    def run(self):
        try:
            self.client.start()
            while not rospy.is_shutdown():
                with self.condition:
                    self.condition.wait_for(lambda: self.stopped or self.pending is not None)
                    if self.stopped:
                        break
                    image, received = self.pending
                    self.pending = None
                output, targets = self.process(image, received)
                if not rospy.is_shutdown():
                    if targets.targets:
                        self.broadcaster.sendTransform([target.transform for target in targets.targets])
                    self.publisher.publish(output)
                    self.tf_targets.publish(targets)
        finally:
            self.client.close()

    def process(self, image, received):
        output = DetectionArray(header=image.header, image_width=image.width, image_height=image.height)
        targets = QrTransformArray(header=image.header)
        stamp = image.header.stamp
        camera_frame = self.camera_frame or image.header.frame_id.strip().lstrip('/')
        if camera_frame != self.last_frame or (self.last_stamp is not None and stamp < self.last_stamp):
            self.tracks.clear()
        self.last_frame, self.last_stamp = camera_frame, stamp
        try:
            gray = self.bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
            centers = decode_centers(gray)
        except (CvBridgeError, cv2.error, ValueError) as error:
            rospy.logwarn_throttle(5.0, 'Cannot decode ODIN image: %s', error)
            return output, targets

        deadline = received + self.query_timeout
        children = self.tracks.update(centers, received)
        for (content, u, v), child in zip(centers, children):
            result = Detection(content=content, center_u=u, center_v=v)
            result.depth_m = result.sync_dt = math.nan
            result.lidar_point.point.x = result.lidar_point.point.y = result.lidar_point.point.z = math.nan
            result.status, response = self.add_depth(result, stamp, deadline)
            output.detections.append(result)
            if response is not None:
                transform = self.camera_transform(response, camera_frame, child)
                if transform is not None:
                    targets.targets.append(QrTransform(content, u, v, transform))
        return output, targets

    def camera_transform(self, response, camera_frame, child):
        if not camera_frame or camera_frame == response.header.frame_id.lstrip('/'):
            rospy.logwarn_throttle(
                5.0, 'Set ~camera_frame to the ODIN camera optical frame to publish QR TF')
            return None
        point = response.camera_point
        if (response.header.stamp.is_zero() or point.z <= 0 or
                not all(math.isfinite(value) for value in (point.x, point.y, point.z))):
            rospy.logwarn_throttle(5.0, 'OYO returned an invalid camera point or depth stamp')
            return None
        transform = TransformStamped()
        transform.header.stamp = response.header.stamp
        transform.header.frame_id = camera_frame
        transform.child_frame_id = child
        transform.transform.translation.x = point.x
        transform.transform.translation.y = point.y
        transform.transform.translation.z = point.z
        transform.transform.rotation.w = 1.0
        return transform

    def add_depth(self, result, stamp, deadline):
        if stamp.is_zero():
            return 'INVALID_IMAGE_STAMP', None

        while not rospy.is_shutdown():
            payload, status = self.client.call(result.center_u, result.center_v, stamp, deadline)
            if payload is None:
                return status, None
            response = QueryPixelResponse().deserialize(payload)
            if response.success:
                if response.image_stamp != stamp:
                    return 'IMAGE_STAMP_MISMATCH', None
                point = response.point
                if (not response.header.frame_id or response.depth <= 0 or
                        not all(math.isfinite(value) for value in
                                (response.depth, point.x, point.y, point.z, response.sync_dt))):
                    return 'INVALID_DEPTH_RESPONSE', None
                result.depth_valid = True
                result.depth_m = response.depth
                result.lidar_point.header = response.header
                result.lidar_point.point = point
                result.sync_dt = response.sync_dt
                return response.status, response

            retry = response.status.startswith(('NO_FRAME', 'FRAME_NOT_FOUND'))
            if not retry or time.monotonic() + 0.03 >= deadline:
                return response.status, None
            time.sleep(0.03)
        return 'SHUTDOWN', None


def main():
    rospy.init_node('uav_odin_qr')
    OdinQr().run()
