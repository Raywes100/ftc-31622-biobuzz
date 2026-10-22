# Limelight vision subsystem POC (BIOBUZZ)

Scope: vision only. Produces the position of our CELL relative to the robot for the motion/firing code. See VISION_ICD.md for the interface contract.

## Files

| File | Purpose |
| --- | --- |
| `TargetVision.java` | The deliverable. Owns the Limelight, outputs a `Target` (x, y, z, bearing, range) in the robot frame. |
| `VisionTestOpMode.java` | Visual feedback test harness: LED and Driver Station show what `TargetVision` reports. |
| `VISION_ICD.md` | Interface contract with the controls team, calibration list, acceptance test. |
| `ball_classifier.py` | Optional Limelight Python pipeline for ball color. Not needed for the target position. |
| `test_classifier_laptop.py` | Runs the classifier on the laptop webcam. |

Both Java files go in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision/`.

## Parts

- REV Control Hub, FTC SDK 12.0 (Android Studio Narwhal 3 Feature Drop or later)
- Limelight 3A, short USB-C cable to a USB port on the Control Hub
- goBILDA RGB Indicator Light (3118-0808-0002) on a servo port, optional
- Laptop on the Control Hub Wi-Fi for the Limelight web UI

## Setup, in order

1. Robot config: Driver Station > Configure Robot > Control Hub > Ethernet Devices > Limelight 3A, named `limelight`. Indicator light as a Servo named `led`.
2. Web UI: laptop on the Control Hub Wi-Fi, open `http://limelight.local:5801`. Update Limelight OS first; updates wipe pipelines, so export pipeline JSON after tuning.
3. Pipeline 1, type AprilTag (Fiducial): family 36h11, tag size 3.25 in (82.55 mm). Field map off.
4. (Optional) Pipeline 0, type Python: paste `ball_classifier.py`.
5. Measure the camera mount and set `CAM_X_IN, CAM_Y_IN, CAM_Z_IN, CAM_PITCH_DEG, CAM_YAW_DEG` in `TargetVision.java`.
6. Fill in the second blue cluster's tag IDs from manual Figure 9-17.

## Run

Deploy, select "Vision Test", Start. Carry the robot toward a printed tag cluster. LED: off = no camera data, white = no alliance tag, orange = target seen, green = bearing within 3 degrees. Y toggles alliance.

## First checks

1. Point the camera straight at a tag 36 in away. Telemetry `Robot-frame xyz` should read roughly (36 + CAM_X_IN, 0, tag height). If x and y are swapped or signs are wrong, the camera-space axis convention needs flipping in `update()`.
2. Run the acceptance test in VISION_ICD.md.

## Rules

- R702: Limelight 3A is the only programmable vision coprocessor allowed.
- R202: LED solid or under 2 Hz.
- R704.D: no video stream to the Driver Station in matches. Web UI is for the pit.
