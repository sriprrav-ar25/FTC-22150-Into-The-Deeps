package org.firstinspires.ftc.teamcode.trainee;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * ShooterTestOpMode
 *
 * Minimal OpMode that:
 * - initializes the webcamFlywheel subsystem
 * - calls update() each loop
 * - requests the required velocity & rpm for tag ID 1
 * - prints telemetry to help tuning
 */
@TeleOp(name = "Shooter Vision Test")
public class WebcamFlywheelOpmode extends OpMode {

    private webcamFlywheel shooterVision; // your upgraded subsystem

    // example: heights in meters (tune these to your robot/field)
    private static final double SHOOTER_HEIGHT_METERS = 0.42;   // shooter exit height above floor (m)
    private static final double TARGET_HEIGHT_METERS = 1.35;    // target (goal) height above floor (m)

    // the AprilTag ID you want to target (change to the ID your field uses)
    private static final int TARGET_TAG_ID = 1;

    @Override
    public void init() {
        // create subsystem and initialize vision
        shooterVision = new webcamFlywheel();
        shooterVision.init(hardwareMap, telemetry);

        telemetry.addLine("Shooter Vision initialized. Waiting for detections...");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ALWAYS call update() each loop so the subsystem refreshes detections
        shooterVision.update();

        // get the specific tag we want to track
        AprilTagDetection tag = shooterVision.getTagBySpecificID(TARGET_TAG_ID);

        // display the telemetry and computed shooter outputs in one place
        shooterVision.displayShooterTelemetry(tag, SHOOTER_HEIGHT_METERS, TARGET_HEIGHT_METERS);

        telemetry.update();
    }

    @Override
    public void stop() {
        shooterVision.stop();
    }
}
