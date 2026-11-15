"""Decode QR codes and return their centers in the received image."""

import time

import cv2
import numpy as np
import zxingcpp


def _corners(barcode):
    position = barcode.position
    return np.float32([(point.x, point.y) for point in (
        position.top_left, position.top_right, position.bottom_right, position.bottom_left)])


def _valid(points, width, height):
    return (
        np.isfinite(points).all()
        and (points >= 0).all()
        and (points < (width, height)).all()
        and cv2.isContourConvex(points)
        and cv2.contourArea(points) >= 4
    )


def _overlap(first, second):
    area, _ = cv2.intersectConvexConvex(first, second)
    return area > 0.65 * min(cv2.contourArea(first), cv2.contourArea(second))


def _rectified(gray, points, deadline):
    # A quiet zone around the warp avoids including nearby texture.
    square = np.float32([(32, 32), (287, 32), (287, 287), (32, 287)])
    transform = cv2.getPerspectiveTransform(points, square)
    patch = cv2.warpPerspective(gray, transform, (320, 320))
    patch[:32, :] = patch[288:, :] = patch[:, :32] = patch[:, 288:] = 255
    for adaptive in (False, True):
        if time.monotonic() >= deadline:
            break
        image = cv2.adaptiveThreshold(
            patch, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 5
        ) if adaptive else patch
        codes = zxingcpp.read_barcodes(image, formats=zxingcpp.BarcodeFormat.QRCode)
        if len(codes) == 1 and codes[0].valid:
            return codes[0].bytes
    return None


def decode_centers(gray):
    height, width = gray.shape
    deadline = time.monotonic() + 0.035
    accepted = []
    conflicts = []
    candidates = []

    def accept(payload, points):
        if not _valid(points, width, height):
            return
        try:
            content = payload.decode('utf-8', errors='strict')
        except UnicodeDecodeError:
            return
        if not content or any(_overlap(points, prior) for prior in conflicts):
            return
        matches = [i for i, (_, prior) in enumerate(accepted) if _overlap(points, prior)]
        if any(accepted[i][0] != content for i in matches):
            conflicts.append(points)
            for i in reversed(matches):
                conflicts.append(accepted.pop(i)[1])
        elif not matches:
            accepted.append((content, points))

    for barcode in zxingcpp.read_barcodes(
        gray, formats=zxingcpp.BarcodeFormat.QRCode, return_errors=True
    ):
        points = _corners(barcode)
        if barcode.valid:
            accept(barcode.bytes, points)
        elif _valid(points, width, height):
            candidates.append(points)

    candidates.sort(key=cv2.contourArea, reverse=True)
    for points in candidates[:4]:
        if time.monotonic() >= deadline:
            break
        if any(_overlap(points, prior) for _, prior in accepted):
            continue
        payload = _rectified(gray, points, deadline)
        if payload is not None:
            accept(payload, points)

    if not accepted and time.monotonic() < deadline:
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 51, 7
        )
        for barcode in zxingcpp.read_barcodes(binary, formats=zxingcpp.BarcodeFormat.QRCode):
            if barcode.valid:
                accept(barcode.bytes, _corners(barcode))

    return [(content, float(u), float(v)) for content, points in accepted
            for u, v in [points.mean(axis=0, dtype=np.float64)]]
