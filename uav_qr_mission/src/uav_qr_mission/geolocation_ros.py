"""ROS data validation, bounded timestamp matching, and installation TF publication."""
from collections import deque
import copy
import math
import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .geolocation import Installation, normalize_quaternion, offset_wgs84


class SampleCache:
    def __init__(self, duration):
        self.duration = duration
        self.samples = deque(maxlen=4096)

    def prune(self, now):
        while self.samples and now - self.samples[0][0] > self.duration:
            self.samples.popleft()

    def add(self, message):
        stamp = message.header.stamp.to_nsec()
        if stamp <= 0:
            return
        now = time.monotonic()
        self.prune(now)
        self.samples.append((now, stamp, message))

    def nearest(self, stamp):
        self.prune(time.monotonic())
        if not self.samples:
            return None
        return min(self.samples, key=lambda sample: (abs(sample[1]-stamp), -sample[1]))[2]


class Geolocation:
    def __init__(self, config):
        self.installation = Installation(config)
        self.fix_topic = config['fix_topic']
        self.attitude_topic = config['attitude_topic']
        self.max_fix_dt = float(config['max_fix_dt_s'])
        self.max_attitude_dt = float(config['max_attitude_dt_s'])
        duration = float(config['cache_duration_s'])
        if (not all(math.isfinite(v) and v > 0 for v in
                    (self.max_fix_dt, self.max_attitude_dt, duration)) or
                duration < max(self.max_fix_dt, self.max_attitude_dt)):
            raise ValueError('Invalid geolocation timestamp/cache limits')
        if not self.fix_topic or not self.attitude_topic:
            raise ValueError('Geolocation input topics must be nonempty')
        self.fixes = SampleCache(duration)
        self.attitudes = SampleCache(duration)
        publish_static = config.get('publish_static_tf', True)
        if not isinstance(publish_static, bool):
            raise ValueError('publish_static_tf must be a boolean')
        if publish_static:
            self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
            messages = []
            for parent, child, xyz, q in self.installation.edges:
                message = TransformStamped()
                message.header.stamp = rospy.Time.now()
                message.header.frame_id = parent
                message.child_frame_id = child
                p, r = message.transform.translation, message.transform.rotation
                p.x, p.y, p.z = xyz
                r.x, r.y, r.z, r.w = q
                messages.append(message)
            self.static_broadcaster.sendTransform(messages)

    def locate(self, report):
        """Return (NavSatFix, fix stamp, attitude stamp), status; never use latest TF."""
        if report.header.frame_id != self.installation.camera:
            return None, 'CAMERA_FRAME_MISMATCH'
        stamp = report.header.stamp.to_nsec()
        fix = self.fixes.nearest(stamp)
        if fix is None:
            return None, 'WAIT_FIX'
        if abs((fix.header.stamp-report.header.stamp).to_sec()) > self.max_fix_dt:
            return None, 'FIX_TIME_MISMATCH'
        if fix.status.status < NavSatStatus.STATUS_FIX:
            return None, 'FIX_INVALID'
        attitude = self.attitudes.nearest(stamp)
        if attitude is None:
            return None, 'WAIT_ATTITUDE'
        if abs((attitude.header.stamp-report.header.stamp).to_sec()) > self.max_attitude_dt:
            return None, 'ATTITUDE_TIME_MISMATCH'
        if attitude.orientation_covariance[0] == -1:
            return None, 'ATTITUDE_INVALID'
        q = attitude.orientation
        try:
            quaternion = normalize_quaternion([q.x, q.y, q.z, q.w])
        except (ValueError, TypeError, OverflowError):
            return None, 'ATTITUDE_INVALID'
        p = report.transform.translation
        try:
            enu = self.installation.antenna_to_target_enu([p.x, p.y, p.z], quaternion)
            lla = offset_wgs84(fix.latitude, fix.longitude, fix.altitude, enu)
            if not all(math.isfinite(value) for value in lla):
                raise ValueError('Nonfinite WGS84 result')
        except (ValueError, TypeError, OverflowError):
            return None, 'FIX_INVALID'
        result = NavSatFix()
        result.header.stamp = report.header.stamp
        result.header.frame_id = 'wgs84'
        result.status = copy.deepcopy(fix.status)
        result.latitude, result.longitude, result.altitude = lla
        # Antenna covariance omits attitude, extrinsic, and depth uncertainty.
        # Leave total QR covariance UNKNOWN rather than copy misleading accuracy.
        result.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return (result, fix.header.stamp, attitude.header.stamp), 'VALID'
