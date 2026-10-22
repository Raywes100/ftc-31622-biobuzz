package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Visual feedback for the vision subsystem. Exercises TargetVision exactly the way the
 * motion/firing code will, and shows the result on an indicator LED and the Driver Station.
 * No drive code here; carry the robot around by hand or push it on a cart.
 *
 * LED (goBILDA RGB Indicator on servo port "led"):
 *   off     no fresh data from the camera
 *   white   camera alive, no alliance tag in view
 *   orange  target found, bearing outside tolerance
 *   green   target found, bearing within tolerance
 * Y toggles alliance.
 */
@TeleOp(name = "Vision Test", group = "Vision")
public class VisionTestOpMode extends LinearOpMode {

    static final boolean HAS_LED = true;
    static final double TOLERANCE_DEG = 3.0;
    static final double LED_OFF = 0.0, LED_ORANGE = 0.333, LED_GREEN = 0.5, LED_WHITE = 1.0;

    @Override
    public void runOpMode() {
        TargetVision vision = new TargetVision();
        vision.init(hardwareMap, TargetVision.Alliance.RED);
        Servo led = HAS_LED ? hardwareMap.get(Servo.class, "led") : null;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready. Y = toggle alliance.");
        telemetry.update();
        waitForStart();
        vision.start();

        boolean lastY = false;
        while (opModeIsActive()) {
            if (gamepad1.y && !lastY) {
                vision.setAlliance(vision.getAlliance() == TargetVision.Alliance.RED
                        ? TargetVision.Alliance.BLUE : TargetVision.Alliance.RED);
            }
            lastY = gamepad1.y;

            vision.update();
            TargetVision.Target t = vision.getTarget();
            boolean cameraAlive = vision.getRawResult() != null && vision.getRawResult().isValid();

            double ledPos;
            if (!cameraAlive) ledPos = LED_OFF;
            else if (!t.valid) ledPos = LED_WHITE;
            else ledPos = Math.abs(t.bearingDeg) <= TOLERANCE_DEG ? LED_GREEN : LED_ORANGE;
            if (led != null) led.setPosition(ledPos);

            telemetry.addData("Alliance", vision.getAlliance());
            telemetry.addData("Camera", cameraAlive ? "ok" : "<b style='color:#e33'>no data</b>");
            if (t.valid) {
                telemetry.addData("Target", "tag %d (%d seen), age %d ms", t.tagId, t.tagCount, t.ageMs);
                telemetry.addData("Bearing", "<b style='color:%s'>%.1f deg %s</b>",
                        Math.abs(t.bearingDeg) <= TOLERANCE_DEG ? "#0c0" : "#f80",
                        t.bearingDeg, t.bearingDeg > 0 ? "(left)" : "(right)");
                telemetry.addData("Range", "%.0f in horizontal", t.rangeIn);
                telemetry.addData("Robot-frame xyz", "%.0f, %.0f, %.0f in", t.xIn, t.yIn, t.zIn);
            } else {
                telemetry.addLine("No alliance tag in view");
            }
            telemetry.update();
        }
        vision.stop();
    }
}
