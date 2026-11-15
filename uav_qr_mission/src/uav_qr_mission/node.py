"""ROS mission progress and the camera-to-report-target transform."""

import copy
import math
import threading
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from uav_odin_qr.msg import DetectionArray, QrTransformArray

from .mission import Mission
from .msg import MissionResult
from .srv import SetTask, SetTaskResponse


REPORT_CHILD = 'uav_qr_report_target'
OBSERVATION_TIMEOUT = 1.0
FRAME_LIMIT = 32


class MissionNode:
    def __init__(self):
        self.result_frame = rospy.get_param('~result_frame', 'arena').strip().lstrip('/')
        if not self.result_frame or self.result_frame == REPORT_CHILD:
            raise ValueError('~result_frame must name a separate result coordinate frame')
        self.mission = Mission()
        self.lock = threading.Lock()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.publisher = rospy.Publisher('~result', MissionResult, queue_size=1, latch=True)
        self.frames = {}
        self.highest_image_stamp = 0
        self.last_tf_stamp = 0
        self.reset_task()
        self.publish()
        self.service = rospy.Service('~set_task', SetTask, self.set_task)
        self.detections_sub = rospy.Subscriber(
            rospy.get_param('~detections_topic', '/uav_odin_qr/detections'),
            DetectionArray, self.on_detections, queue_size=8)
        self.targets_sub = rospy.Subscriber(
            rospy.get_param('~tf_targets_topic', '/uav_odin_qr/tf_targets'),
            QrTransformArray, self.on_targets, queue_size=8)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)

    def reset_task(self):
        self.frames.clear()
        self.task_cutoff = self.highest_image_stamp
        self.last_paired_stamp = self.task_cutoff
        self.position = None
        self.report_tf = None
        self.report_received = 0.0
        self.position_candidate = None

    def set_task(self, request):
        with self.lock:
            accepted, status = self.mission.set_task(
                request.mission_id, request.target_ids, request.position_target_id)
            if status == 'ACCEPTED':
                self.reset_task()
                self.publish()
            return SetTaskResponse(accepted, status)

    def publish(self):
        result = MissionResult(
            mission_id=self.mission.id, target_ids=list(self.mission.targets),
            position_target_id=self.mission.position_target,
            found=[target in self.mission.found for target in self.mission.targets])
        if self.position is not None:
            result.position = self.position
            result.position_valid = True
        if self.report_tf is not None:
            result.report_tf = self.report_tf
            result.report_tf_valid = True
        result.complete = bool(self.mission.id and all(result.found) and result.position_valid)
        if not self.mission.id:
            result.status = 'WAIT_TASK'
        elif result.complete:
            result.status = 'COMPLETE'
        elif all(result.found):
            result.status = 'WAIT_POSITION'
        else:
            result.status = 'SEARCHING'
        self.publisher.publish(result)

    def prune_frames(self, now):
        for key, frame in list(self.frames.items()):
            if now - frame[0] > OBSERVATION_TIMEOUT:
                del self.frames[key]
        while len(self.frames) >= FRAME_LIMIT:
            del self.frames[next(iter(self.frames))]

    def on_detections(self, message):
        self.receive_frame(message, is_detections=True)

    def on_targets(self, message):
        self.receive_frame(message, is_detections=False)

    def receive_frame(self, message, is_detections):
        with self.lock:
            key = message.header.stamp.to_nsec()
            self.highest_image_stamp = max(self.highest_image_stamp, key)
            if not self.mission.id or key <= self.task_cutoff:
                return
            now = time.monotonic()
            self.prune_frames(now)
            changed = False
            if is_detections:
                count = len(self.mission.found)
                self.mission.observe(message.detections)
                changed = len(self.mission.found) != count
                if key > self.last_paired_stamp and len(self.mission.position_detections(message.detections)) != 1:
                    self.last_paired_stamp = key
                    changed = self.report_tf is not None or changed
                    self.report_tf = None
            if key > self.last_paired_stamp:
                frame = self.frames.setdefault(key, [now, None, None])
                frame[1 if is_detections else 2] = message
                if frame[1] is not None and frame[2] is not None:
                    del self.frames[key]
                    self.last_paired_stamp = key
                    self.match_frame(frame)
                    changed = True
            if changed:
                self.publish()

    def match_frame(self, frame):
        received, detections, targets = frame
        source = None
        if detections.header.frame_id == targets.header.frame_id:
            source = self.mission.select_transform(detections.detections, targets.targets)
        if source is None:
            self.report_tf = None
            return
        stamp = source.header.stamp.to_nsec()
        camera = source.header.frame_id
        point = source.transform.translation
        if (not camera or camera == REPORT_CHILD or not source.child_frame_id or
                stamp <= 0 or point.z <= 0 or not all(math.isfinite(v) for v in (point.x, point.y, point.z))):
            self.report_tf = None
            return
        if stamp <= self.last_tf_stamp:
            return
        report = copy.deepcopy(source)
        report.child_frame_id = REPORT_CHILD
        self.broadcaster.sendTransform(report)
        self.last_tf_stamp = stamp
        self.report_tf, self.report_received = report, received
        # Keep the candidate while the field TF catches up to its measurement time.
        if self.position is None and self.position_candidate is None:
            self.position_candidate = (received, report)
            self.try_position()

    def try_position(self):
        if self.position_candidate is None:
            return False
        received, report = self.position_candidate
        if time.monotonic() - received > OBSERVATION_TIMEOUT:
            self.position_candidate = None
            return False
        point = PointStamped()
        point.header = copy.copy(report.header)
        point.point.x = report.transform.translation.x
        point.point.y = report.transform.translation.y
        point.point.z = report.transform.translation.z
        if point.header.frame_id != self.result_frame:
            try:
                transform = self.buffer.lookup_transform(
                    self.result_frame, point.header.frame_id, point.header.stamp, rospy.Duration(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return False
            point = do_transform_point(point, transform)
            point.header.stamp = report.header.stamp
        if not all(math.isfinite(v) for v in (point.point.x, point.point.y, point.point.z)):
            self.position_candidate = None
            return False
        self.position, self.position_candidate = point, None
        return True

    def tick(self, _event):
        with self.lock:
            changed = self.try_position()
            if self.report_tf is not None and time.monotonic() - self.report_received > OBSERVATION_TIMEOUT:
                self.report_tf = None
                changed = True
            if changed:
                self.publish()


def main():
    rospy.init_node('uav_qr_mission')
    node = MissionNode()
    rospy.spin()
