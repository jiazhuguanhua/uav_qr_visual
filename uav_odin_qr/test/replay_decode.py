#!/usr/bin/env python3
"""Offline 2D regression on the ODIN undistorted image track of ROS1 bags."""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge

from uav_odin_qr.decoder import decode_centers


def measure(path, expected):
    bridge = CvBridge()
    longest_gap = gap_frames = decoded = total = wrong = 0
    timings = []
    with rosbag.Bag(str(path), 'r') as bag:
        for _, image, _ in bag.read_messages(topics=['/odin1/image/undistorted']):
            total += 1
            start = time.monotonic()
            gray = bridge.imgmsg_to_cv2(image, desired_encoding='mono8')
            result = decode_centers(gray)
            timings.append((time.monotonic() - start) * 1000)
            contents = [row[0] for row in result]
            correct = expected in contents
            wrong += sum(content != expected for content in contents)
            decoded += correct
            if correct:
                gap_frames = 0
            else:
                gap_frames += 1
                longest_gap = max(longest_gap, gap_frames)
    return {
        'bag': path.name, 'frames': total, 'correct': decoded, 'wrong_codes': wrong,
        'longest_miss_frames': longest_gap,
        'processing_ms_p95': float(np.percentile(timings, 95)) if timings else None
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', type=Path, nargs='+')
    parser.add_argument('--expected', default='QR001')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    for path in args.bags:
        print(json.dumps(measure(path, args.expected), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
