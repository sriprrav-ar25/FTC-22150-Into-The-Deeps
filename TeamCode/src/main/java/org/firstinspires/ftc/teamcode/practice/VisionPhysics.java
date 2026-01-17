package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.mechanisms.webcam;

import java.util.List;

@TeleOp(name = "Vision + Physics Calculator", group = "TeleOp")
public class VisionPhysics extends OpMode {
    // Physics constants (copied from your PhysicsEquation)
    private final double GRAVITY = 9.81;
    private final double FLYWHEEL_RADIUS = 0.0762; // m (3 in)
    private final double SHOOTER_HEIGHT = 0.237;   // m (237 mm)
    private final double GOAL_HEIGHT = 0.9845;
    private final double Y_MAX = GOAL_HEIGHT - SHOOTER_HEIGHT + 0.127; // m (5 in headspace)

    private webcam cam; // your webcam class (unchanged)
    private Telemetry telemetryRef;

    // If you want to target a particular AprilTag id, set that here (or -1 to use first detected)
    private final int TARGET_TAG_ID = 20;

    // IMPORTANT: adjust this conversion if your VisionPortal actually returns inches instead of cm.
    // Your aprilTag processor builder used DistanceUnit.CM, so we assume ftcPose.* are in cm.
    // Conversion: inches -> meters => divide by 100.0
    private final double DISTANCE_TO_METERS = 0.0254;

    @Override
    public void init() {
        telemetryRef = telemetry;
        cam = new webcam();
        cam.init(hardwareMap, telemetryRef);
        telemetryRef.addLine("VisionPhysicsCalculator initialized");
        telemetryRef.update();
    }

    @Override
    public void loop() {
        // update vision detections
        cam.update();
        List<AprilTagDetection> detections = cam.getDetectedTags();

        if (detections == null || detections.size() == 0) {
            telemetryRef.addLine("No tags detected.");
            telemetryRef.update();
            return;
        }

        // pick detection: specific ID if set, otherwise first
        AprilTagDetection chosen = null;
        if (TARGET_TAG_ID >= 0) {
            chosen = cam.getTagBySpecificID(TARGET_TAG_ID);
            if (chosen == null) {
                telemetryRef.addData("Tag", "ID %d not found", TARGET_TAG_ID);
                telemetryRef.update();
                return;
            }
        } else {
            chosen = detections.get(0);
        }

        // Extract raw values from detection.pose
        // NOTE: these field names follow what your display method used:
        // detection.ftcPose.z and detection.ftcPose.pitch
        double rawZ = chosen.ftcPose.z;      // raw distance is in inches by default
        double pitchDeg = chosen.ftcPose.pitch; // degrees (builder used AngleUnit.DEGREES)

        // Convert to meters & radians (theta used in trig must be radians)
        double rangeMeters = rawZ * DISTANCE_TO_METERS;
        double thetaRadians = Math.toRadians(pitchDeg);

        // Physics calculation (same formula you used in PhysicsEquation)
        double cosTheta = Math.cos(thetaRadians);
        double cosThetaSq = cosTheta * cosTheta;
        double tanTheta = Math.tan(thetaRadians);

        double numerator = GRAVITY * Math.pow(rangeMeters, 2);
        double denominator = 2.0 * cosThetaSq *
                (rangeMeters * tanTheta - Y_MAX);

        double requiredOmega = Double.NaN;
        boolean valid = true;
        if (denominator <= 0 || numerator / denominator < 0) {
            valid = false;
        } else {
            requiredOmega = Math.sqrt(numerator / denominator) / FLYWHEEL_RADIUS; // rad/s
        }

        // Telemetry output
        telemetryRef.addLine("=== Vision -> Physics ===");
        telemetryRef.addData("Tag ID", chosen.id);
        telemetryRef.addData("Raw z (in)", "%.2f", rawZ);
        telemetryRef.addData("Range (m)", "%.3f", rangeMeters);
        telemetryRef.addData("Pitch (deg)", "%.2f", pitchDeg);
        telemetryRef.addData("Theta (rad)", "%.4f", thetaRadians);

        if (!valid || Double.isNaN(requiredOmega) || Double.isInfinite(requiredOmega)) {
            telemetryRef.addLine("Calculation invalid (check target geometry / angle).");
            telemetryRef.addData("Denominator", "%.6f", denominator);
            telemetryRef.addData("Numerator/Denom", "%.6f", (denominator == 0 ? Double.NaN : numerator/denominator));
        } else {
            telemetryRef.addData("Required Omega (rad/s)", "%.2f", requiredOmega);
            double requiredRPM = requiredOmega * 60.0 / (2.0 * Math.PI);
            telemetryRef.addData("Required RPM", "%.1f", requiredRPM);
        }

        telemetryRef.update();
    }

    @Override
    public void stop() {
        cam.stop();
    }
}
