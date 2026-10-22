# Limelight SnapScript pipeline: classify BIOBUZZ balls by color in one frame.
# Paste into a Python pipeline (slot 0) in the Limelight web UI.
#
# Output llpython (12 numbers, read by VisualFeedbackPOC.java):
#   [yFound, yTx, yTy, yArea,  rFound, rTx, rTy, rArea,  bFound, bTx, bTy, bArea]
#   found: 1 or 0; tx/ty: degrees from image center (+x right, +y up); area: % of image.
#
# HSV ranges start from the TRC BIOBUZZ guide. Tune in the venue: lower the top of the
# yellow hue range if green shows up; red may need the 170-180 band added.

import cv2
import numpy as np

# OpenCV hue is 0-179. (hLo, hHi, sLo, sHi, vLo, vHi)
YELLOW = [(18, 35, 110, 255, 100, 255)]
RED    = [(0, 8, 110, 255, 100, 255), (170, 179, 110, 255, 100, 255)]
BLUE   = [(100, 125, 110, 255, 100, 255)]

MIN_AREA_FRAC = 0.0005   # ignore blobs under 0.05 % of the image
MIN_FULLNESS  = 0.5      # contour area / bounding box area; a ball is round
HFOV_DEG = 54.5          # Limelight 3A horizontal FOV
VFOV_DEG = 42.0          # Limelight 3A vertical FOV

KERNEL = np.ones((3, 3), np.uint8)


def mask_for(hsv, ranges):
    m = None
    for (hl, hh, sl, sh, vl, vh) in ranges:
        part = cv2.inRange(hsv, (hl, sl, vl), (hh, sh, vh))
        m = part if m is None else cv2.bitwise_or(m, part)
    m = cv2.erode(m, KERNEL, iterations=1)
    m = cv2.dilate(m, KERNEL, iterations=2)
    return m


def largest_ball(mask, img_w, img_h):
    """Return (found, tx, ty, areaPct, contour) for the largest round blob."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0.0
    img_area = float(img_w * img_h)
    for c in contours:
        a = cv2.contourArea(c)
        if a < MIN_AREA_FRAC * img_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if h == 0 or a / float(w * h) < MIN_FULLNESS:
            continue
        aspect = w / float(h)
        if aspect < 0.6 or aspect > 1.6:
            continue
        if a > best_area:
            best, best_area = c, a
    if best is None:
        return 0, 0.0, 0.0, 0.0, None
    x, y, w, h = cv2.boundingRect(best)
    cx, cy = x + w / 2.0, y + h / 2.0
    tx = (cx - img_w / 2.0) / (img_w / 2.0) * (HFOV_DEG / 2.0)
    ty = -(cy - img_h / 2.0) / (img_h / 2.0) * (VFOV_DEG / 2.0)
    return 1, tx, ty, 100.0 * best_area / img_area, best


def runPipeline(image, llrobot):
    h, w = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    out = []
    biggest, biggest_area = None, 0.0
    for name, ranges, color in (("Y", YELLOW, (0, 220, 255)),
                                ("R", RED, (0, 0, 255)),
                                ("B", BLUE, (255, 80, 0))):
        found, tx, ty, area, cnt = largest_ball(mask_for(hsv, ranges), w, h)
        out += [found, tx, ty, area]
        if found:
            x, y, bw, bh = cv2.boundingRect(cnt)
            cv2.rectangle(image, (x, y), (x + bw, y + bh), color, 2)
            cv2.putText(image, "%s %.1f%%" % (name, area), (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            if area > biggest_area:
                biggest, biggest_area = cnt, area

    # Limelight uses the returned contour for its own tx/ty/ta; return the biggest ball.
    if biggest is None:
        biggest = np.array([[]])
    return biggest, image, out
