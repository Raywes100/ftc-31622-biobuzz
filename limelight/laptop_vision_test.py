# Test the Limelight with no Control Hub: camera plugged straight into the laptop over USB-C.
#
# Reads the Limelight's JSON results over HTTP, applies the same camera-to-robot transform as
# TargetVision.java, and shows the visual feedback in a window (green = locked, orange = target
# seen, white = no alliance tag, grey = no data). Use it to check pipelines, tag IDs, and the
# axis convention before the Java version ever runs.
#
#   pip install opencv-python numpy requests
#   python laptop_vision_test.py              # uses http://limelight.local
#   python laptop_vision_test.py 172.28.0.1   # or give the camera's address
# Keys: a = toggle alliance, q = quit.

import sys
import time
import math
import requests
import cv2
import numpy as np

HOST = sys.argv[1] if len(sys.argv) > 1 else "limelight.local"
RESULTS_URL = "http://%s:5807/results" % HOST
STREAM_URL = "http://%s:5800/stream.mjpg" % HOST

# ---- Same constants as TargetVision.java. Measure these on the real robot. -------------------
CAM_X_IN, CAM_Y_IN, CAM_Z_IN = 6.0, 0.0, 12.0      # lens position in robot frame (fwd, left, up)
CAM_PITCH_DEG, CAM_YAW_DEG = 30.0, 0.0             # camera tilt up, twist left
AIM_TOLERANCE_DEG = 3.0
MAX_AGE_MS = 150
RED_TAGS = {0, 1, 2, 3, 4, 5, 6, 7}
BLUE_TAGS = {38, 39, 40, 41}
IN_PER_M = 39.37


def camera_to_robot(x_right_m, y_down_m, z_fwd_m):
    """Limelight camera space (x right, y down, z forward, meters) -> robot frame (inches)."""
    f, l, u = z_fwd_m * IN_PER_M, -x_right_m * IN_PER_M, -y_down_m * IN_PER_M
    cp, sp = math.cos(math.radians(CAM_PITCH_DEG)), math.sin(math.radians(CAM_PITCH_DEG))
    f1, u1 = f * cp - u * sp, f * sp + u * cp
    cy, sy = math.cos(math.radians(CAM_YAW_DEG)), math.sin(math.radians(CAM_YAW_DEG))
    x = f1 * cy - l * sy + CAM_X_IN
    y = f1 * sy + l * cy + CAM_Y_IN
    z = u1 + CAM_Z_IN
    return x, y, z


def read_target(alliance_red):
    """Returns dict or None, mirroring TargetVision.Target."""
    try:
        j = requests.get(RESULTS_URL, timeout=0.2).json()
    except Exception:
        return {"camera": False}
    ours = RED_TAGS if alliance_red else BLUE_TAGS
    tags = [t for t in j.get("Fiducial", []) if int(t.get("fID", -1)) in ours]
    out = {"camera": True, "pipeline": j.get("pID"), "valid": False, "seen": len(j.get("Fiducial", [])),
           "python": j.get("PythonOut")}
    if not tags:
        return out
    best = max(tags, key=lambda t: t.get("ta", 0))
    pose = best.get("t6t_cs")
    if not pose or len(pose) < 3:
        return out
    x, y, z = camera_to_robot(pose[0], pose[1], pose[2])
    out.update(valid=True, tag=int(best["fID"]), count=len(tags), x=x, y=y, z=z,
               bearing=math.degrees(math.atan2(y, x)), range=math.hypot(x, y),
               tx=best.get("tx", 0.0))
    return out


def main():
    alliance_red = True
    cap = cv2.VideoCapture(STREAM_URL)
    have_stream = cap.isOpened()
    if not have_stream:
        print("No MJPEG stream at %s; running without video." % STREAM_URL)

    while True:
        t = read_target(alliance_red)
        if have_stream:
            ok, frame = cap.read()
            if not ok:
                frame = np.zeros((480, 640, 3), np.uint8)
        else:
            frame = np.zeros((480, 640, 3), np.uint8)

        if not t["camera"]:
            color, line1 = (120, 120, 120), "NO DATA from %s" % HOST
        elif not t["valid"]:
            color, line1 = (255, 255, 255), "no %s tag (%d tags in view)" % ("red" if alliance_red else "blue", t["seen"])
        elif abs(t["bearing"]) <= AIM_TOLERANCE_DEG:
            color, line1 = (0, 200, 0), "LOCKED  tag %d" % t["tag"]
        else:
            color, line1 = (0, 140, 255), "turn %s  tag %d" % ("left" if t["bearing"] > 0 else "right", t["tag"])

        cv2.rectangle(frame, (0, 0), (frame.shape[1], 70), color, -1)
        cv2.putText(frame, line1, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        if t.get("valid"):
            line2 = "bearing %.1f deg  range %.0f in  xyz %.0f, %.0f, %.0f in  (raw tx %.1f)" % (
                t["bearing"], t["range"], t["x"], t["y"], t["z"], t["tx"])
            cv2.putText(frame, line2, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            print("\r" + line1 + "  " + line2 + "      ", end="")
        else:
            print("\r" + line1 + "      ", end="")
        py = t.get("python")
        if py and len(py) >= 12:
            names = ("POLLEN", "NECTAR_RED", "NECTAR_BLUE")
            found = [names[i] for i in range(3) if py[i * 4] > 0]
            cv2.putText(frame, "balls: " + (", ".join(found) if found else "none"), (10, frame.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, "alliance: %s  (a to toggle, q to quit)" % ("RED" if alliance_red else "BLUE"),
                    (frame.shape[1] - 330, frame.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("Limelight vision test", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("a"):
            alliance_red = not alliance_red
        if not have_stream:
            time.sleep(0.05)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
