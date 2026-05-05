import cv2
import numpy as np
import math

image_height, image_width, ch = 0, 0, 0
robot_height_m = 30 * 0.0254
mount_angle_deg = 30
MOUNT_ANGLE_RAD = math.radians(mount_angle_deg)

_half_w = 0.0
_half_h = 0.0
_sx = 0.0
_sy = 0.0

LOWER_YELLOW = np.array([20, 80, 70],  dtype=np.uint8)
UPPER_YELLOW = np.array([40, 255, 255], dtype=np.uint8)

LOWER_WHITE = np.array([200, 200, 200],  dtype=np.uint8)
UPPER_WHITE = np.array([255, 255, 255],  dtype=np.uint8)

def runPipeline(image, llrobot):
    global image_height, image_width, ch
    global _half_w, _half_h, _sx, _sy

    if image_height == 0:
        image_height, image_width, ch = image.shape
        _half_w = image_width  / 2.0
        _half_h = image_height / 2.0
        _sx     = 62.5 / image_width
        _sy     = 48.9 / image_height

    blurred     = cv2.blur(image,   (11, 11))
    blurred     = cv2.blur(blurred, (11, 11))
    hsv         = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    white_mask = cv2.inRange(blurred, LOWER_WHITE, UPPER_WHITE)
    yellow_mask =cv2.bitwise_or( yellow_mask, white_mask )

    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    max_score    = 0
    best_contour = np.array([[]])
    llpython     = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 25:
            continue

        # Simple centroid from contour moments — no weighted V channel needed
        M = cv2.moments(cnt)
        if M['m00'] <= 0:
            continue

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        curr_score, x, y = score(cx, cy, area)

        if curr_score > max_score:
            max_score    = curr_score
            best_contour = cnt
            llpython     = [x, y]

        cv2.circle(blurred, (cx, cy), 5, (0, 0, 255), -1)

    return best_contour, blurred, llpython


def get_cord_relative_to_bot(x, y):
    tx, ty = input_to_angle(x, y)
    dist = robot_height_m / math.tan(MOUNT_ANGLE_RAD + math.radians(-ty))
    left = math.tan(math.radians(-tx)) * dist
    return (
        dist, left
    )


def input_to_angle(x, y):
    x = (-x + _half_w) * _sx
    y = (-y + _half_h) * _sy
    return x, y


def score(x, y, area):
    rx, ry = get_cord_relative_to_bot(x, y)

    # Horizontal offset from robot centerline in meters
    horiz_offset = abs(ry)

    # Estimate distance from area: bigger apparent area = closer ball
    # area ~ k / dist^2  →  dist ~ sqrt(k / area)
    # k is a tunable constant representing expected ball area at 1m
    K = 5000.0
    est_dist = math.sqrt(K / max(area, 1))

    if est_dist > 10:
        return 0, None, None

    # Score: prefer large (close) contours, penalize horizontal offset
    area_score   = math.sqrt(area) / 10.0
    center_score = 1.0 / (horiz_offset + 0.3)

    final_score = area_score + center_score

    print(f"est_dist={est_dist:.2f}, area_score={area_score:.2f}, center_score={center_score:.2f}")

    return int(math.ceil(final_score)), rx, ry