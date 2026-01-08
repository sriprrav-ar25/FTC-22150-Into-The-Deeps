package org.firstinspires.ftc.teamcode.trainee;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
 * - Handles AprilTag vision (VisionPortal + AprilTagProcessor)
 * - Computes required launch velocity (m/s) using the tangent-only formula
 * - Converts velocity -> RPM
 * - Contains a software PIDF controller to convert a target RPM -> motor power using encoder feedback
 *
 * Important unit conventions:
 * - Distances (vision outputs) are set to METERS in the AprilTagProcessor
 * - Angles from the tag are in DEGREES (we convert to radians before trig)
 */
public class webcamFlywheelPIDF {
    // ---------------------- Vision ----------------------
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;

    // ---------------------- Physics constants & config ----------------------
    private static final double GRAVITY = 9.81;           // m/s^2
    private static final double SHOOTER_OFFSET_DEG = 10.0; // shooter fixed +10° relative to camera axis
    private static final double FUDGE_FACTOR_DEFAULT = 1.10; // empirical multiplier (tune on-field)

    // ---------------------- Flywheel/Motor config (tune these to your robot) ----------------------
    // Wheel radius (m) used to convert linear velocity to RPM
    private double wheelRadiusMeters = 0.05; // CHANGE to your wheel radius

    // Encoder: ticks (counts) per wheel revolution (depends on your encoder/motor)
    // Example: NeveRest Orbital 20: 560 ticks/rev ; REV HD Hex: 28 or 560 etc. Set to your hardware.
    private int ticksPerWheelRevolution = 28 * 1; // CHANGE to actual ticks per wheel rotation

    // Maximum safe power and RPM (for clamping)
    private double MAX_POWER = 1.0;
    private double MIN_POWER = -1.0;
    private double MAX_RPM = 6000.0; // just a high clamp; tune for your wheel/motor

    // ---------------------- PIDF controller variables (software PID) ----------------------
    // Default gains — you WILL tune these on your robot
    private double kP = 0.003;     // proportional gain (RPM -> power)
    private double kI = 0.00001;   // integral gain
    private double kD = 0.0005;    // derivative gain
    private double kF = 0.0;       // feedforward term (you can set to 0 or use simple linear FF)

    // controller state
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = 0L;

    // Anti-windup / integral clamp
    private double integralClamp = 5000.0;

    // target RPM
    private volatile double targetRPM = 0.0;

    // motor reference (must be set/initialized by OpMode via initMotor)
    private DcMotorEx flywheelMotor = null;

    // last measured rpm (for telemetry)
    private double lastMeasuredRPM = 0.0;

    // fudge factor (empirical compensation on top of physics)
    private double fudgeFactor = FUDGE_FACTOR_DEFAULT;

    // ---------------------- Initialization ----------------------
    /**
     * init - sets up vision processing
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Build AprilTag processor (meters + degrees)
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();

        // initialize timing for PID
        lastTimeNs = System.nanoTime();
    }

    /**
     * initMotor - provide the flywheel motor so the subsystem can control it.
     * - The motor should be configured in the robot config and passed as a DcMotorEx.
     * - ticksPerWheelRev must be set to match the encoder counts for one wheel revolution.
     */
    public void initMotor(DcMotorEx motor, int ticksPerWheelRev, double wheelRadiusMeters) {
        this.flywheelMotor = motor;
        this.ticksPerWheelRevolution = ticksPerWheelRev;
        this.wheelRadiusMeters = wheelRadiusMeters;

        // recommended motor setup
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Do NOT set power here; PID loop will set it each update
    }

    // ---------------------- Vision update ----------------------
    /**
     * update() - must be called repeatedly from OpMode.loop()
     * refreshes detection list
     */
    public void update() {
        List<AprilTagDetection> latest = aprilTagProcessor.getDetections();
        detectedTags.clear();
        if (latest != null) detectedTags.addAll(latest);
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public AprilTagDetection getTagBySpecificID(int id) {
        for (AprilTagDetection d : detectedTags) {
            if (d.id == id) return d;
        }
        return null;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (telemetry == null) return;
        if (detectedId == null) {
            telemetry.addLine("No tag (displayDetectionTelemetry)");
            return;
        }
        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.2f %6.2f %6.2f  (m)",
                    detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.2f %6.2f %6.2f  (deg)",
                    detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.2f %6.2f %6.2f  (m, deg, deg)",
                    detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    // ---------------------- Shooter physics ----------------------
    /**
     * calculateLaunchVelocity
     * returns required linear velocity (m/s) or -1 if impossible
     *
     * Uses:
     *  - tag.ftcPose.range (meters)
     *  - tag.ftcPose.elevation (degrees relative to camera axis)
     *  - shooterHeightMeters, targetHeightMeters (m)
     *  - shooter fixed offset (+SHOOTER_OFFSET_DEG above camera axis)
     */
    public double calculateLaunchVelocity(AprilTagDetection tag, double shooterHeightMeters, double targetHeightMeters) {
        if (tag == null) return -1.0;

        // 1) straight-line range (m) from camera to tag
        double rangeMeters = tag.ftcPose.range;

        // 2) elevation angle relative to camera axis (deg)
        double cameraElevationDeg = tag.ftcPose.elevation;

        // 3) compute horizontal distance x (m) from camera geometry: x = range * cos(elevation)
        double cameraElevationRad = Math.toRadians(cameraElevationDeg);
        double x = rangeMeters * Math.cos(cameraElevationRad);

        // 4) vertical displacement y = targetHeight - shooterHeight (m)
        double y = targetHeightMeters - shooterHeightMeters;

        // 5) true shooter angle relative to horizontal (deg): shooter = camera axis + mechanical offset
        double shooterAngleDeg = SHOOTER_OFFSET_DEG + cameraElevationDeg;

        // ensure shooter cannot go below mechanical offset
        if (shooterAngleDeg < SHOOTER_OFFSET_DEG) shooterAngleDeg = SHOOTER_OFFSET_DEG;

        double theta = Math.toRadians(shooterAngleDeg);
        double tanTheta = Math.tan(theta);

        // feasibility check: denominator must be positive
        if (x * tanTheta <= y) {
            // shot impossible
            if (telemetry != null) {
                telemetry.addLine("Shot impossible (x * tanTheta <= y)");
                telemetry.addData("x", x);
                telemetry.addData("tanTheta", tanTheta);
                telemetry.addData("y", y);
            }
            return -1.0;
        }

        // tangent-only projectile formula
        double numerator = GRAVITY * x * x * (1.0 + tanTheta * tanTheta);
        double denominator = 2.0 * (x * tanTheta - y);

        if (denominator <= 0) return -1.0;

        double v = Math.sqrt(numerator / denominator);

        // apply empirical fudge factor to correct real-world losses
        v *= fudgeFactor;

        return v;
    }

    /**
     * velocityToRPM - convert linear exit velocity (m/s) to RPM of the wheel
     * Uses v = omega * r ; omega rad/s = v / r ; RPM = omega * 60 / (2*pi)
     */
    public double velocityToRPM(double vMetersPerSecond) {
        if (vMetersPerSecond <= 0 || wheelRadiusMeters <= 0) return 0.0;
        double omega = vMetersPerSecond / wheelRadiusMeters; // rad/s
        double rpm = omega * (60.0 / (2.0 * Math.PI));
        return rpm;
    }

    // ---------------------- PIDF control (software) ----------------------

    /**
     * setTargetRPM - main external API to request a desired wheel RPM
     */
    public void setTargetRPM(double rpm) {
        // clamp to robot's maximum RPM safety
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        if (rpm < 0) rpm = 0;
        this.targetRPM = rpm;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getLastMeasuredRPM() {
        return lastMeasuredRPM;
    }

    public void setFudgeFactor(double fudge) {
        this.fudgeFactor = fudge;
    }

    public double getFudgeFactor() { return this.fudgeFactor; }

    public void setPIDGains(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }

    /**
     * readMotorRPM - calculates wheel RPM from encoder delta between calls
     *
     * Method:
     * - read current ticks via getCurrentPosition()
     * - compute ticks / sec from delta ticks / delta time
     * - convert ticks/sec -> RPM
     */
    private double readMotorRPM() {
        if (flywheelMotor == null) return 0.0;

        // get current time and position
        long nowNs = System.nanoTime();
        double nowSec = nowNs / 1e9;
        double dt = (nowNs - lastTimeNs) / 1e9;
        if (dt <= 0) dt = 1e-6; // protect divide by zero

        int ticks = flywheelMotor.getCurrentPosition(); // encoder ticks since power-on
        // we need to store lastTicks between calls; use a field
        // to do that, store as a static variable inside class:
        // (we'll add lastTicks field to the class)
        // (ensure it's initialized properly)
        return computeRPMFromTicks(ticks, dt);
    }

    // internal fields for tick-based RPM computation
    private int lastTicks = 0;
    private boolean firstTickRead = true;

    /**
     * computeRPMFromTicks - returns RPM and updates lastTicks
     */
    private double computeRPMFromTicks(int ticks, double dtSeconds) {
        if (firstTickRead) {
            // On the first call we don't have a previous sample; initialize and return 0
            lastTicks = ticks;
            firstTickRead = false;
            lastTimeNs = System.nanoTime();
            return 0.0;
        }

        int deltaTicks = ticks - lastTicks;
        lastTicks = ticks;

        // ticks per second
        double ticksPerSec = deltaTicks / dtSeconds;

        // revolutions per second = ticksPerSec / ticksPerRev
        double revPerSec = ticksPerSec / (double) ticksPerWheelRevolution;

        // RPM = rev/sec * 60
        double rpm = revPerSec * 60.0;

        // store last measured rpm for telemetry
        lastMeasuredRPM = rpm;

        // update lastTimeNs for next dt computation
        lastTimeNs = System.nanoTime();

        return rpm;
    }

    /**
     * updatePIDAndApply - call this frequently (every loop) to update the PID controller
     * and set motor power accordingly. This method uses the encoder-based RPM measurement.
     *
     * Returns the power value applied (for telemetry).
     */
    public double updatePIDAndApply() {
        if (flywheelMotor == null) {
            // no motor connected - nothing to do
            return 0.0;
        }

        // read dt using stored lastTimeNs
        long nowNs = System.nanoTime();
        double dt = (nowNs - lastTimeNs) / 1e9;
        if (dt <= 0) dt = 1e-6;

        // read motor RPM using encoder delta
        // Note: computeRPMFromTicks updates lastTicks and lastTimeNs inside it.
        // But we need dt for PID integral; to keep simple we compute rpm then compute dt here again.
        int currentTicks = flywheelMotor.getCurrentPosition();
        double measuredRPM = computeRPMFromTicks(currentTicks, dt);

        // compute PID error (target - measured)
        double error = targetRPM - measuredRPM;

        // integral with anti-windup
        integral += error * dt;
        if (integral > integralClamp) integral = integralClamp;
        if (integral < -integralClamp) integral = -integralClamp;

        // derivative
        double derivative = 0.0;
        if (dt > 0) derivative = (error - lastError) / dt;

        // simple feedforward term: map targetRPM to a baseline power fraction
        // You can make this more sophisticated (e.g. kV*targetVel) but this linear proportion is simple
        double powerFF = (targetRPM / MAX_RPM); // scale 0..1 roughly
        // Apply gain kF to this feedforward (kF is small if you want mostly PID)
        double ffTerm = kF * powerFF;

        // PID output = P + I + D + FF
        double outputPower = kP * error + kI * integral + kD * derivative + ffTerm;

        // clamp output to safe motor powers
        if (outputPower > MAX_POWER) outputPower = MAX_POWER;
        if (outputPower < MIN_POWER) outputPower = MIN_POWER;

        // apply to motor
        flywheelMotor.setPower(outputPower);

        // update PID state
        lastError = error;

        // store lastMeasuredRPM for telemetry
        lastMeasuredRPM = measuredRPM;

        // update lastTimeNs for the next iteration
        lastTimeNs = nowNs;

        return outputPower;
    }

    // convenience: stop motor
    public void stopMotor() {
        if (flywheelMotor != null) {
            flywheelMotor.setPower(0.0);
        }
    }

    // ---------------------- Telemetry helper ----------------------
    public void displayAllTelemetry(AprilTagDetection tag, double shooterHeightMeters, double targetHeightMeters) {
        // show detection info
        displayDetectionTelemetry(tag);

        // physics -> v -> rpm
        double v = calculateLaunchVelocity(tag, shooterHeightMeters, targetHeightMeters);
        double rpm = 0;
        if (v > 0) rpm = velocityToRPM(v);

        telemetry.addData("Calculated v (m/s)", String.format("%.2f", v));
        telemetry.addData("Calculated RPM", String.format("%.0f", rpm));
        telemetry.addData("TargetRPM (set)", String.format("%.0f", targetRPM));
        telemetry.addData("MeasuredRPM", String.format("%.0f", lastMeasuredRPM));
        telemetry.addData("PID P I D F", String.format("%.6f %.6f %.6f %.6f", kP, kI, kD, kF));
    }

    // ---------------------- Shutdown ----------------------
    public void stop() {
        if (visionPortal != null) visionPortal.close();
        stopMotor();
    }
}
