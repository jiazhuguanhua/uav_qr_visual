#!/usr/bin/env python3
"""Exercise the real ROS topic and service contract on an isolated test master."""

import math
import queue
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Time as TimeMessage
from tf2_msgs.msg import TFMessage
from uav_oyo.srv import QueryPixel, QueryPixelResponse

from uav_odin_qr.msg import DetectionArray, QrTransformArray


# ODIN-QR, version 1, with two quiet-zone modules.
QR = """
0000000000000000000000000
0000000000000000000000000
0011111110010110111111100
0010000010011100100000100
0010111010110110101110100
0010111010010100101110100
0010111010001010101110100
0010000010000010100000100
0011111110101010111111100
0000000000110110000000000
0011101111111101100010000
0011110100101000100000000
0010010111110010001001100
0001111001111000100111000
0011000110101010101011100
0000000000100101010101000
0011111110100101110101000
0010000010101111011010100
0010111010101101110110100
0010111010010000110101000
0010111010111010001100100
0010000010110000101011100
0011111110100010100110100
0000000000000000000000000
0000000000000000000000000
"""


def main():
    rospy.init_node('uav_odin_qr_contract', anonymous=True)
    outputs = queue.Queue()
    tf_targets = queue.Queue()
    tf_messages = queue.Queue()
    requests = queue.Queue()
    mode = 'good'

    def query(request):
        nonlocal mode
        requests.put(request)
        response = QueryPixelResponse(image_stamp=request.image_stamp)
        if mode == 'not_synced':
            mode = 'good'
            response.status = 'FRAME_NOT_FOUND'
            return response
        if mode == 'no_depth' or (mode == 'left_no_depth' and request.u < 200):
            response.status = 'NO_DEPTH'
            return response
        if mode == 'hung':
            time.sleep(1.5)
        response.success = True
        response.status = 'OK'
        if mode == 'mismatch':
            response.image_stamp += rospy.Duration(0, 1)
        response.depth = 3.0
        response.header.frame_id = 'lidar'
        response.depth_stamp = response.header.stamp = request.image_stamp - rospy.Duration(0, 5_000_000)
        response.sync_dt = 0.005
        response.point.x, response.point.y, response.point.z = 1.0, 2.0, 3.0
        response.camera_point.x = request.u / 100.0
        response.camera_point.y = request.v / 100.0
        response.camera_point.z = response.depth
        return response

    service = rospy.Service('/uav_odin_qr_test/query_pixel', QueryPixel, query)
    publisher = rospy.Publisher('/uav_odin_qr_test/image', Image, queue_size=1)
    subscriber = rospy.Subscriber('/uav_odin_qr/detections', DetectionArray, outputs.put)
    targets_sub = rospy.Subscriber('/uav_odin_qr/tf_targets', QrTransformArray, tf_targets.put)
    tf_sub = rospy.Subscriber('/tf', TFMessage, tf_messages.put)
    until = time.monotonic() + 8
    while not all(peer.get_num_connections() for peer in (publisher, subscriber, targets_sub, tf_sub)):
        assert time.monotonic() < until, 'ROS topics did not connect'
        time.sleep(0.03)

    bridge = CvBridge()
    modules = np.uint8([[0 if module == '1' else 255 for module in row] for row in QR.split()])
    symbol = cv2.resize(modules, (200, 200), interpolation=cv2.INTER_NEAREST)
    gray = np.full((300, 400), 255, dtype=np.uint8)
    gray[60:260, 80:280] = symbol

    def send(stamp, image, frame=''):
        message = bridge.cv2_to_imgmsg(cv2.cvtColor(image, cv2.COLOR_GRAY2RGB), encoding='rgb8')
        message.header.stamp = stamp
        message.header.frame_id = frame
        publisher.publish(message)
        result = outputs.get(timeout=3)
        assert result.header.stamp == stamp and result.header.frame_id == frame
        assert (result.image_width, result.image_height) == (400, 300)
        targets = tf_targets.get(timeout=3)
        assert targets.header.stamp == stamp
        if not frame:
            assert not targets.targets
        return result.detections, targets.targets

    stamp = rospy.Time(123, 500_000_000)
    result, = send(stamp, gray)[0]
    assert result.content == 'ODIN-QR' and result.depth_valid and result.depth_m == 3.0, str(result)
    assert abs(result.center_u - 180) <= 2 and abs(result.center_v - 160) <= 2
    assert result.lidar_point.header.frame_id == 'lidar'
    assert result.lidar_point.header.stamp == stamp - rospy.Duration(0, 5_000_000)
    assert (result.lidar_point.point.x, result.lidar_point.point.y, result.lidar_point.point.z) == (1, 2, 3)
    request = requests.get_nowait()
    assert request.image_stamp == stamp and request.u == result.center_u and request.v == result.center_v

    for mode, expected in (('no_depth', 'NO_DEPTH'), ('mismatch', 'IMAGE_STAMP_MISMATCH'),
                           ('hung', 'QUERY_TIMEOUT')):
        stamp += rospy.Duration(1)
        result, = send(stamp, gray)[0]
        assert result.status == expected and not result.depth_valid and math.isnan(result.depth_m)
        assert all(math.isnan(value) for value in (
            result.lidar_point.point.x, result.lidar_point.point.y, result.lidar_point.point.z))
        assert requests.get_nowait().image_stamp == stamp

    mode = 'good'
    result, = send(stamp + rospy.Duration(1), gray)[0]
    assert result.depth_valid and result.status == 'OK'
    requests.get_nowait()
    result, = send(rospy.Time(), gray)[0]
    assert result.status == 'INVALID_IMAGE_STAMP' and not result.depth_valid
    assert not send(stamp + rospy.Duration(2), np.full_like(gray, 255))[0]
    assert requests.empty(), 'Zero-stamp and empty images must not query OYO'

    mode = 'not_synced'
    stamp += rospy.Duration(3)
    result, = send(stamp, gray)[0]
    assert result.depth_valid
    assert requests.get_nowait().image_stamp == requests.get_nowait().image_stamp == stamp

    service.shutdown()
    stamp += rospy.Duration(1)
    result, = send(stamp, gray)[0]
    assert not result.depth_valid and result.status.startswith('SERVICE_UNAVAILABLE')
    service = rospy.Service('/uav_odin_qr_test/query_pixel', QueryPixel, query)
    stamp += rospy.Duration(1)
    result, = send(stamp, gray)[0]
    assert result.depth_valid and requests.get_nowait().image_stamp == stamp

    smaller = cv2.resize(modules, (120, 120), interpolation=cv2.INTER_NEAREST)
    pair = np.full_like(gray, 255)
    pair[90:210, 20:140] = smaller
    pair[90:210, 260:380] = smaller
    frame = 'odin_test_camera_optical'
    stamp += rospy.Duration(1)
    detections, targets = send(stamp, pair, frame)
    assert len(detections) == len(targets) == 2
    ordered = sorted(targets, key=lambda target: target.center_u)
    children = [target.transform.child_frame_id for target in ordered]
    assert len(set(children)) == 2
    for target in ordered:
        transform = target.transform
        assert target.content == 'ODIN-QR'
        assert transform.header.frame_id == frame
        assert transform.header.stamp == stamp - rospy.Duration(0, 5_000_000)
        assert transform.transform.translation.x == target.center_u / 100.0
        assert transform.transform.translation.y == target.center_v / 100.0
        assert transform.transform.translation.z == 3.0
        assert transform.transform.rotation.w == 1.0
    assert {tf.child_frame_id for tf in tf_messages.get(timeout=3).transforms} == set(children)

    stamp += rospy.Duration(1)
    mode = 'left_no_depth'
    detections, targets = send(stamp, np.roll(pair, 3, axis=1), frame)
    assert len(detections) == 2 and sum(d.depth_valid for d in detections) == 1
    target, = targets
    assert target.center_u > 200 and target.transform.child_frame_id == children[1]
    only_tf, = tf_messages.get(timeout=3).transforms
    assert only_tf.child_frame_id == children[1]

    # Exercise the real worker's pause/trigger/resume path, not just its callback.
    mode = 'good'
    while not requests.empty():
        requests.get_nowait()
    continuous = rospy.Publisher('/uav_odin_qr/continuous', Bool, queue_size=1)
    trigger = rospy.Publisher('/uav_odin_qr/capture_after', TimeMessage, queue_size=1)
    until = time.monotonic()+3
    while not continuous.get_num_connections() or not trigger.get_num_connections():
        assert time.monotonic() < until
        time.sleep(.02)

    def publish_only(value):
        message = bridge.cv2_to_imgmsg(cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB), encoding='rgb8')
        message.header.stamp = value
        message.header.frame_id = frame
        publisher.publish(message)
        time.sleep(.2)

    continuous.publish(Bool(False))
    time.sleep(.1)
    stamp += rospy.Duration(2)
    publish_only(stamp)
    assert outputs.empty() and requests.empty(), 'Paused camera must not decode/query depth'
    trigger.publish(TimeMessage(stamp+rospy.Duration(2)))
    time.sleep(.1)
    publish_only(stamp+rospy.Duration(1))
    assert outputs.empty() and requests.empty(), 'Trigger must reject pre-window RGB'
    result, = send(stamp+rospy.Duration(2), gray, frame)[0]
    assert result.depth_valid and requests.get_nowait().image_stamp == stamp+rospy.Duration(2)
    publish_only(stamp+rospy.Duration(3))
    assert outputs.empty() and requests.empty(), 'One trigger must process exactly one frame'
    continuous.publish(Bool(True))
    time.sleep(.1)
    assert send(stamp+rospy.Duration(4), gray, frame)[0][0].depth_valid
    service.shutdown()
    print('PASS: exact stamps, depth failures/recovery, multiple TF children, pause/fresh-trigger/resume')


if __name__ == '__main__':
    main()
