package org.firstinspires.ftc.teamcode.trainee;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * webcamFlywheel
 *
 * This class is a **vision + shooter-math subsystem**:
 * - handles the VisionPortal + AprilTagProcessor setup
 * - stores the most recent detections
 * - provides methods that compute required launch velocity (m/s)
 *   and convert that velocity to flywheel RPM
 *
 * IMPORTANT:
 * - All distances returned here are in METERS (we set processor to DistanceUnit.METER).
 * - Angles from aprilTag.ftcPose.* are in DEGREES (we convert to radians for trig).
 * - Camera axis is defined as 0° (the camera's forward direction). The shooter is fixed
 *   at SHOOTER_OFFSET_DEG above the camera axis (10° per your requirement).
 */
public class webcamFlywheel {
    // ---------------------- Vision objects ----------------------
    private AprilTagProcessor aprilTagProcessor;         // AprilTag detection and pose estimation
    private VisionPortal visionPortal;                   // VisionPortal wrapper (camera + processors)
    private List<AprilTagDetection> detectedTags = new ArrayList<>(); // latest stable detections
    private Telemetry telemetry;                         // telemetry reference injected from OpMode

    // ---------------------- Physical constants & config ----------------------
    private static final double GRAVITY = 9.81;          // gravity (m / s^2)
    private static final double SHOOTER_OFFSET_DEG = 10.0; // shooter is fixed +10° relative to camera axis
    private static final double FUDGE_FACTOR = 1.10;     // empirical correction factor (tune on robot)
    private static final double WHEEL_RADIUS_METERS = 0.05; // example wheel radius (m) -> change to your wheel

    // ---------------------- Initialization ----------------------
    /**
     * init
     * @param hardwareMap FTC HardwareMap to get the webcam
     * @param telemetry OpMode telemetry for debug output
     *
     * This method configures the AprilTag processor and creates the VisionPortal.
     * We set output units to METERS so all distance fields (range, x, y, z) are in meters.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        // keep telemetry reference for use in this class
        this.telemetry = telemetry;

        // build the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)              // draw text ID on camera preview
                .setDrawTagOutline(true)         // draw outline box
                .setDrawAxes(true)               // draw axes for pose visualization
                .setDrawCubeProjection(true)     // draw cube projection on tag (nice visual)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES) // USE METERS and DEGREES
                .build();

        // build the VisionPortal and attach processor
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")); // name must match robot config
        builder.setCameraResolution(new Size(640, 480));                 // resolution (tweak if needed)
        builder.addProcessor(aprilTagProcessor);                         // add the AprilTag processor

        // finalize VisionPortal
        visionPortal = builder.build();
    }

    // ---------------------- Update loop ----------------------
    /**
     * update()
     * - should be called repeatedly from the OpMode loop()
     * - reads detections from the AprilTagProcessor and places them in our local list
     */
    public void update() {
        // copy the detections into our list (avoid holding same reference)
        // This is defensive: many pipelines return a mutable list internally.
        List<AprilTagDetection> latest = aprilTagProcessor.getDetections();
        detectedTags.clear();
        if (latest != null) detectedTags.addAll(latest);
    }

    // ---------------------- Accessors ----------------------
    /**
     * getDetectedTags()
     * returns the most recent list of AprilTagDetection objects (may be empty)
     */
    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    /**
     * getTagBySpecificID
     * helper that returns the first detection with the requested ID, or null if not found
     */
    public AprilTagDetection getTagBySpecificID(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }

    /**
     * displayDetectionTelemetry
     * Nicely print detection info to telemetry for debugging.
     * Uses METERS for distances and DEGREES for angles because of our processor settings.
     */
    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {
            telemetry.addLine("No tag provided (null)");
            return;
        }
        // If metadata is present (friendly name), show it; otherwise show unknown ID
        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            // ftcPose.x/y/z are in meters (we set DistanceUnit.METER)
            telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f  (m)",
                    detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            // pitch/roll/yaw in degrees
            telemetry.addLine(String.format("PRY %6.2f %6.2f %6.2f  (deg)",
                    detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            // range (m), bearing (deg), elevation (deg)
            telemetry.addLine(String.format("RBE %6.2f %6.2f %6.2f  (m, deg, deg)",
                    detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            // if no metadata, still show a minimal readout
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    // ---------------------- Shooter physics ----------------------
    /**
     * calculateLaunchVelocity
     *
     * Uses the tangent-only projectile formula (your preferred form).
     *
     * INPUTS (we require these):
     *  - tag: the AprilTagDetection for the target
     *  - shooterHeightMeters: height of the shooter exit above ground (m)
     *  - targetHeightMeters: height of the target above ground (m)
     *
     * RETURNS:
     *  - required linear launch speed (m/s) to hit the target
     *  - returns -1 if shot is impossible (physically invalid / division by zero)
     *
     * NOTES:
     *  - We compute horizontal distance x from tag.ftcPose.range and tag.ftcPose.elevation
     *    because range is the straight-line distance from camera to tag and elevation is
     *    the angle from camera forward axis up to the target (in degrees).
     *  - Shooter angle relative to horizontal is computed as:
     *        shooterAngleDeg = SHOOTER_OFFSET_DEG + cameraElevationDeg
     *    because the shooter is fixed +10° above camera axis per your spec.
     */
    public double calculateLaunchVelocity(AprilTagDetection tag,
                                          double shooterHeightMeters,
                                          double targetHeightMeters) {
        // safety: null tag => nothing to compute
        if (tag == null) {
            telemetry.addLine("NO Tag detected");
            return -1;
        }

        // ---- 1) Extract and convert raw vision readings ----
        // tag.ftcPose.range is straight-line distance from camera to tag (meters, because we set output units)
        double rangeMeters = tag.ftcPose.range; // meters


        // ---- 2) Compute horizontal distance x (meters) ----
        // geometry: x = range * cos(elevation)
        // We convert the elevation to radians for Math.cos
        double cameraElevationRad = Math.toRadians(tag.ftcPose.elevation);
        double x = rangeMeters * Math.cos(cameraElevationRad);

        // ---- 3) Compute vertical displacement y (meters) ----
        // y is targetHeight - shooterHeight (positive if target is above shooter exit)
        double y = targetHeightMeters - shooterHeightMeters;

        // ---- 4) Compute true shooter angle relative to HORIZONTAL (degrees -> radians) ----
        // camera axis = 0° ; shooter is mechanically +SHOOTER_OFFSET_DEG above camera
        double shooterAngleDeg = SHOOTER_OFFSET_DEG + tag.ftcPose.elevation;

        // enforce mechanical minimum (shooter cannot go below SHOOTER_OFFSET_DEG)
        if (shooterAngleDeg < SHOOTER_OFFSET_DEG) shooterAngleDeg = SHOOTER_OFFSET_DEG;

        // convert to radians for trig functions
        double theta = Math.toRadians(shooterAngleDeg);

        // ---- 5) Precompute tan(theta) and safety check ----
        double tanTheta = Math.tan(theta);

        // If denominator would be <= 0, shot impossible (x*tanTheta <= y)
        // Because that would require infinite or imaginary velocity
        if (x * tanTheta <= y) {
            telemetry.addLine("Shot impossible at this angle (x*tanTheta <= y)");
            telemetry.addData("x", x);
            telemetry.addData("tanTheta", tanTheta);
            telemetry.addData("y", y);
            return -1;
        }

        // ---- 6) Apply projectile formula (tangent-only form) ----
        // v = sqrt( g * x^2 * (1 + tan^2(theta)) / (2 * (x*tan(theta) - y)) )
        double numerator = GRAVITY * Math.pow(x,2) * (1.0 + Math.pow(tanTheta,2));
        double denominator = 2.0 * (x * tanTheta - y);

        // small safety check (should not be <= 0 due to earlier guard)
        if (denominator <= 0) {
            telemetry.addLine("Denominator invalid unexpectedly");
            return -1;
        }

        double velocity = Math.sqrt(numerator / denominator);

        // ---- 7) Apply empirical fudge factor to account for losses (wheel slip, drag, compression) ----
        velocity = velocity * FUDGE_FACTOR;

        // return the required linear launch speed in meters/second
        return velocity;
    }

    /**
     * velocityToRPM
     *
     * Converts a linear projectile exit velocity (m/s) to a wheel RPM
     * using the relation v = omega * r ; omega (rad/s) = v / r ; RPM = omega * 60 / (2*pi)
     *
     * This is an approximation: real exit speed depends on wheel contact, compression, spin.
     */
    public double velocityToRPM(double velocityMetersPerSecond, double wheelRadiusMeters) {
        if (velocityMetersPerSecond <= 0 || wheelRadiusMeters <= 0) return -1;
        double omegaRadPerSec = velocityMetersPerSecond / wheelRadiusMeters;
        double rpm = omegaRadPerSec * (60.0 / (2.0 * Math.PI));
        return rpm;
    }

    // convenience method using the internal example wheel radius constant
    public double velocityToRPM(double velocityMetersPerSecond) {
        return velocityToRPM(velocityMetersPerSecond, WHEEL_RADIUS_METERS);
    }

    // ---------------------- Telemetry helper that prints shooter outputs ----------------------
    public void displayShooterTelemetry(AprilTagDetection tag, double shooterHeightMeters, double targetHeightMeters) {
        // Show detection info (range/elevation)
        displayDetectionTelemetry(tag);

        // Compute and show required velocity/rpm if possible
        double v = calculateLaunchVelocity(tag, shooterHeightMeters, targetHeightMeters);
        if (v > 0) {
            telemetry.addData("Required speed (m/s)", String.format("%.2f", v));
            double rpm = velocityToRPM(v);
            telemetry.addData("Target wheel RPM", String.format("%.0f", rpm));
        } else {
            telemetry.addLine("No valid speed computed (invalid/none)");
        }
    }

    // ---------------------- Shutdown ----------------------
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
