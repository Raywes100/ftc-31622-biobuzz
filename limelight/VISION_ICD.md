# Vision subsystem interface (ICD)

Owner: vision. Consumer: motion and firing control. Agree on this before either side writes against it.

## What vision provides

`TargetVision.getTarget()` returns a `Target` snapshot, refreshed by `update()` once per control loop.

| Field | Type | Meaning | Units |
| --- | --- | --- | --- |
| `valid` | boolean | true only when every other field may be used | |
| `xIn`, `yIn`, `zIn` | double | position of the tag center in the robot frame | in |
| `bearingDeg` | double | `atan2(y, x)`; positive = target is to the left; add to robot heading to get field bearing | deg |
| `rangeIn` | double | horizontal distance to the tag center | in |
| `tagId` | int | tag used for the solve | |
| `tagCount` | int | alliance tags visible in that frame; 2+ means a good solve | |
| `ageMs` | long | time since the camera frame | ms |

Robot frame: origin at robot center on the floor, x forward, y left, z up, right-handed. Same frame as odometry; `bearingDeg` has the same sign as heading.

## Guarantees

- `valid` is false if: no frame in the last 150 ms, wrong pipeline, no alliance tag in view, or the pose solve failed.
- Position refers to the tag center, not the CELL opening center. Offset from tag center to opening center is a fixed per-tag constant; apply it on the firing side or ask vision to add it once the cluster geometry is measured.
- Update rate: 100 Hz poll of the camera; new frames at the camera's rate (~30-90 fps depending on resolution). Reading `getTarget()` costs nothing.
- Vision never commands motors. Alliance is set by the OpMode at init and can be toggled.

## Calibration vision owns

- `CAM_X_IN, CAM_Y_IN, CAM_Z_IN`: lens position in robot frame, measured with a ruler.
- `CAM_PITCH_DEG, CAM_YAW_DEG`: mount angles.
- Limelight pipeline: 36h11, tag size 3.25 in, pipeline index 1.
- Blue tag ID list for the second cluster.

## Acceptance test

Place the robot at a marked spot. Tape measure from robot center to a tag center on the ground. Compare `rangeIn` and `bearingDeg` at three ranges (30, 60, 90 in) and three bearings (0, +20, -20 deg). Pass: range within 2 in, bearing within 1 deg. Then carry the robot in a circle and confirm `valid` drops cleanly when the tag leaves view and returns within 150 ms when it reappears.

## Out of scope for this piece

Ball color classification (separate pipeline, `ball_classifier.py`), odometry fusion, shoot-on-the-move prediction. Those can be added behind the same `Target` interface without changing the consumer.
