# Run ball_classifier.py on the laptop webcam before putting it on the Limelight.
#   pip install opencv-python numpy
#   python test_classifier_laptop.py [camera_index]
# Shows the annotated frame and prints the 12-number llpython array. Press q to quit.

import sys
import cv2
import ball_classifier as bc

cam = int(sys.argv[1]) if len(sys.argv) > 1 else 0
cap = cv2.VideoCapture(cam)
if not cap.isOpened():
    sys.exit("no camera at index %d" % cam)

while True:
    ok, frame = cap.read()
    if not ok:
        break
    frame = cv2.resize(frame, (640, 480))
    _, annotated, out = bc.runPipeline(frame, [])
    y, r, b = out[0:4], out[4:8], out[8:12]
    print("Y %d tx %6.1f area %5.2f | R %d tx %6.1f area %5.2f | B %d tx %6.1f area %5.2f"
          % (y[0], y[1], y[3], r[0], r[1], r[3], b[0], b[1], b[3]), end="\r")
    cv2.imshow("ball_classifier", annotated)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
