"""Registered QR registry, stationary capture windows, and measurement-time positions."""
import copy
import math
import threading
import time

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Time as TimeMessage
from tf2_geometry_msgs import do_transform_point
from uav_odin_qr.msg import DetectionArray, QrTransformArray

from .mission import Mission
from .geolocation_ros import Geolocation
from .msg import MissionResult, QrTargetResult
from .srv import (SetTask, SetTaskResponse, CaptureQr, CaptureQrResponse,
                  FinishSurvey, FinishSurveyResponse)

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
        self.geolocation = Geolocation(rospy.get_param('~geolocation'))
        self.publisher = rospy.Publisher('~result', MissionResult, queue_size=1, latch=True)
        self.vision_mode = rospy.Publisher('/uav_odin_qr/continuous', Bool, queue_size=1, latch=True)
        self.vision_capture = rospy.Publisher('/uav_odin_qr/capture_after', TimeMessage, queue_size=1)
        self.highest_image_stamp = 0
        self.last_tf_stamp = 0
        self.reset_task()
        self.publish()
        self.services = [
            rospy.Service('~set_task', SetTask, self.set_task),
            rospy.Service('~capture', CaptureQr, self.capture),
            rospy.Service('~finish_survey', FinishSurvey, self.finish_survey)]
        self.subs = [
            rospy.Subscriber(rospy.get_param('~detections_topic', '/uav_odin_qr/detections'),
                             DetectionArray, self.on_detections, queue_size=8),
            rospy.Subscriber(rospy.get_param('~tf_targets_topic', '/uav_odin_qr/tf_targets'),
                             QrTransformArray, self.on_targets, queue_size=8),
            rospy.Subscriber(self.geolocation.fix_topic, NavSatFix,
                             self.on_fix, queue_size=100),
            rospy.Subscriber(self.geolocation.attitude_topic, Imu, self.on_attitude, queue_size=200)]
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)

    def reset_task(self):
        self.frames = {}
        self.task_cutoff = self.highest_image_stamp
        self.last_paired_stamp = self.task_cutoff
        self.position = None
        self.position_candidate = None
        self.report_tf = None
        self.report_received = 0.0
        self.records = {}
        self.candidates = {}
        for content in self.mission.targets:
            record = QrTargetResult(content=content,
                                    is_drop_target=content == self.mission.position_target,
                                    status='WAIT_OBSERVATION')
            record.position.status.status = NavSatStatus.STATUS_NO_FIX
            record.position.latitude = record.position.longitude = record.position.altitude = math.nan
            self.records[content] = record
        self.survey_finished = False
        self.used_capture_ids = set()
        self.capture_id = ''
        self.capture_status = 'IDLE'
        self.capture_done = False
        self.capture_success = False
        self.capture_stamp = rospy.Time(0)
        self.capture_after = 0
        self.capture_deadline = 0.0

    def on_fix(self, message):
        with self.lock:
            self.geolocation.fixes.add(message)
            if self.try_locations():
                self.publish()

    def on_attitude(self, message):
        with self.lock:
            self.geolocation.attitudes.add(message)
            if self.try_locations():
                self.publish()

    def set_task(self, request):
        with self.lock:
            accepted, status = self.mission.set_task(
                request.mission_id, request.target_ids, request.position_target_id,
                request.capture_only)
            if status == 'ACCEPTED':
                self.reset_task()
                self.vision_mode.publish(Bool(not self.mission.capture_only))
                self.publish()
            return SetTaskResponse(accepted, status)

    def capture(self, request):
        # This service only starts/cancels work. It never waits for an image or RTK.
        # BT polls the matching capture_id on the heartbeat/result topic.
        with self.lock:
            if not self.mission.id or request.mission_id != self.mission.id:
                return CaptureQrResponse(False, 'MISSION_MISMATCH')
            if request.command == request.CANCEL:
                # Remember cancellation even if an in-flight START RPC arrives later.
                if request.capture_id != self.capture_id:
                    if request.capture_id:
                        self.used_capture_ids.add(request.capture_id)
                    return CaptureQrResponse(True, 'CANCELLED_BEFORE_START')
                if not self.capture_done:
                    self.end_capture(False, 'CANCELLED')
                    self.publish()
                return CaptureQrResponse(True, self.capture_status)
            if request.command != request.START or not self.mission.capture_only:
                return CaptureQrResponse(False, 'CAPTURE_MODE_REQUIRED')
            if not request.capture_id or request.not_before.is_zero():
                return CaptureQrResponse(False, 'INVALID_CAPTURE')
            if not math.isfinite(request.timeout_s) or not 0 < request.timeout_s <= 10:
                return CaptureQrResponse(False, 'INVALID_TIMEOUT')
            if (rospy.Time.now()-request.not_before).to_sec() > request.timeout_s:
                return CaptureQrResponse(False, 'REQUEST_EXPIRED')
            if self.survey_finished:
                return CaptureQrResponse(False, 'SURVEY_FINISHED')
            if request.capture_id == self.capture_id:
                return CaptureQrResponse(True, self.capture_status)
            if self.capture_id and not self.capture_done:
                return CaptureQrResponse(False, 'BUSY')
            if request.capture_id in self.used_capture_ids:
                return CaptureQrResponse(False, 'CAPTURE_ID_REUSED')
            self.capture_id = request.capture_id
            self.used_capture_ids.add(request.capture_id)
            self.capture_status = 'WAIT_FRAME'
            self.capture_done = self.capture_success = False
            self.capture_stamp = rospy.Time(0)
            self.capture_after = max(request.not_before.to_nsec(), self.highest_image_stamp + 1)
            self.capture_deadline = time.monotonic() + request.timeout_s
            self.frames.clear()
            self.candidates.clear()
            self.report_tf = None
            self.trigger_vision(self.capture_after)
            self.publish()
            return CaptureQrResponse(True, 'ACCEPTED')

    def end_capture(self, success, status):
        self.capture_done, self.capture_success = True, success
        self.capture_status = status
        self.frames.clear()
        for content in self.candidates:
            self.records[content].status = status
        self.candidates.clear()
        self.trigger_vision(0)
        if not success:
            self.position_candidate = None
            self.report_tf = None

    def trigger_vision(self, nanoseconds):
        stamp = rospy.Time(nanoseconds//1000000000, nanoseconds%1000000000)
        self.vision_capture.publish(TimeMessage(stamp))

    def finish_survey(self, request):
        with self.lock:
            if not self.mission.id or request.mission_id != self.mission.id:
                return FinishSurveyResponse(False, 'MISSION_MISMATCH', self.snapshot())
            if self.capture_id and not self.capture_done:
                return FinishSurveyResponse(False, 'CAPTURE_BUSY', self.snapshot())
            self.survey_finished = True
            self.candidates.clear()
            self.frames.clear()
            self.publish()
            return FinishSurveyResponse(True, 'FINISHED', self.snapshot())

    def snapshot(self):
        result = MissionResult(
            mission_id=self.mission.id, target_ids=list(self.mission.targets),
            position_target_id=self.mission.position_target,
            found=[target in self.mission.found for target in self.mission.targets])
        result.header.stamp = rospy.Time.now()
        probe = TransformStamped()
        probe.header.stamp = result.header.stamp
        probe.header.frame_id = self.geolocation.installation.camera
        probe.transform.translation.z = 1.0
        _, result.geolocation_input_status = self.geolocation.locate(probe)
        if self.position is not None:
            result.position, result.position_valid = self.position, True
        if self.report_tf is not None:
            result.report_tf, result.report_tf_valid = self.report_tf, True
        for content, record in self.records.items():
            record.found = content in self.mission.found
        result.targets = [copy.deepcopy(self.records[c]) for c in self.mission.targets]
        result.all_localized = bool(result.targets and all(r.position_valid for r in result.targets))
        selected = self.records.get(self.mission.position_target)
        if selected is not None:
            result.wgs84_position = selected.position
            result.wgs84_position_valid = selected.position_valid
            result.wgs84_status = selected.status
            result.wgs84_fix_stamp, result.wgs84_attitude_stamp = selected.fix_stamp, selected.attitude_stamp
        else:
            result.wgs84_status = 'WAIT_TASK'
            result.wgs84_position.status.status = NavSatStatus.STATUS_NO_FIX
            result.wgs84_position.latitude = result.wgs84_position.longitude = result.wgs84_position.altitude = math.nan
        # Preserve the old arena completion contract. Survey/UGV use the new fields.
        result.complete = bool(self.mission.id and all(result.found) and result.position_valid)
        if not self.mission.id:
            result.status = 'WAIT_TASK'
        elif result.complete:
            result.status = 'COMPLETE'
        elif all(result.found):
            result.status = 'WAIT_POSITION'
        else:
            result.status = 'SEARCHING'
        result.survey_finished = self.survey_finished
        result.capture_id, result.capture_status = self.capture_id, self.capture_status
        result.capture_done, result.capture_success = self.capture_done, self.capture_success
        result.capture_stamp = self.capture_stamp
        return result

    def publish(self):
        self.publisher.publish(self.snapshot())
        self.last_publish = time.monotonic()

    def on_detections(self, message):
        self.receive_frame(message, True)

    def on_targets(self, message):
        self.receive_frame(message, False)

    def receive_frame(self, message, is_detections):
        with self.lock:
            key = message.header.stamp.to_nsec()
            self.highest_image_stamp = max(self.highest_image_stamp, key)
            if not self.mission.id or key <= self.task_cutoff or self.survey_finished:
                return
            if self.mission.capture_only:
                if (not self.capture_id or self.capture_done or not self.capture_stamp.is_zero()
                        or key < self.capture_after):
                    return
                if time.monotonic() >= self.capture_deadline:
                    self.end_capture(False, 'TIMEOUT')
                    self.publish()
                    return
            if key <= self.last_paired_stamp:
                return
            now = time.monotonic()
            self.frames = {k: f for k, f in self.frames.items() if now-f[0] <= OBSERVATION_TIMEOUT}
            if len(self.frames) >= FRAME_LIMIT:
                self.frames.pop(next(iter(self.frames)))
            frame = self.frames.setdefault(key, [now, None, None])
            frame[1 if is_detections else 2] = message
            if frame[1] is None or frame[2] is None:
                return
            del self.frames[key]
            if frame[1].header.frame_id != frame[2].header.frame_id:
                return
            if self.mission.capture_only and any(
                    t.content in self.records and t.transform.header.stamp.to_nsec() < self.capture_after
                    for t in frame[2].targets):
                # A synchronized depth may slightly predate the first fresh image.
                # Skip that pair and wait for another frame inside the same capture.
                self.trigger_vision(key+1)
                return
            self.last_paired_stamp = key
            self.mission.observe(frame[1].detections)
            if self.mission.capture_only:
                self.capture_stamp = message.header.stamp
                self.capture_status = 'LOCALIZING'
            self.match_frame(frame)
            self.try_locations()
            self.publish()

    def match_frame(self, frame):
        received, detections, targets = frame
        selected_report = None
        for content, record in self.records.items():
            source = self.mission.select_transform(detections.detections, targets.targets, content)
            if source is None:
                if not record.position_valid and any(d.content == content for d in detections.detections):
                    record.status = 'NO_UNIQUE_VALID_DEPTH'
                continue
            p = source.transform.translation
            if (not source.header.frame_id or source.header.frame_id == REPORT_CHILD or
                    not source.child_frame_id or source.header.stamp.is_zero() or p.z <= 0 or
                    not all(math.isfinite(v) for v in (p.x, p.y, p.z))):
                continue
            # Image freshness alone is insufficient: OYO's depth measurement must
            # also belong to this stationary window, never a cached in-flight frame.
            if self.mission.capture_only and source.header.stamp.to_nsec() < self.capture_after:
                if not record.position_valid:
                    record.status = 'DEPTH_BEFORE_CAPTURE'
                continue
            if not record.position_valid and content not in self.candidates:
                self.candidates[content] = (received, copy.deepcopy(source))
            if content == self.mission.position_target:
                selected_report = copy.deepcopy(source)
        self.report_tf = None
        if selected_report is not None and selected_report.header.stamp.to_nsec() > self.last_tf_stamp:
            selected_report.child_frame_id = REPORT_CHILD
            self.broadcaster.sendTransform(selected_report)
            self.report_tf, self.report_received = selected_report, received
            self.last_tf_stamp = selected_report.header.stamp.to_nsec()
            if self.position is None and self.position_candidate is None:
                self.position_candidate = (received, selected_report)
                self.try_position()

    def try_locations(self):
        changed = False
        for content, (received, report) in list(self.candidates.items()):
            record = self.records[content]
            previous = record.status
            if time.monotonic()-received > OBSERVATION_TIMEOUT:
                record.status = 'OBSERVATION_EXPIRED'
                del self.candidates[content]
            else:
                position, record.status = self.geolocation.locate(report)
                if position is not None:
                    record.position, record.fix_stamp, record.attitude_stamp = position
                    record.position_valid, record.revision = True, 1
                    del self.candidates[content]
                    changed = True
            changed = previous != record.status or changed
        if (self.mission.capture_only and self.capture_id and not self.capture_done
                and not self.capture_stamp.is_zero() and not self.candidates):
            self.end_capture(True, 'CAPTURED')
            changed = True
        return changed

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
            changed = self.try_locations() or changed
            now = time.monotonic()
            if self.capture_id and not self.capture_done and now >= self.capture_deadline:
                self.end_capture(False, 'TIMEOUT')
                changed = True
            if self.report_tf is not None and now-self.report_received > OBSERVATION_TIMEOUT:
                self.report_tf = None
                changed = True
            if changed or now-self.last_publish >= 0.5:
                self.publish()


def main():
    rospy.init_node('uav_qr_mission')
    MissionNode()
    rospy.spin()
