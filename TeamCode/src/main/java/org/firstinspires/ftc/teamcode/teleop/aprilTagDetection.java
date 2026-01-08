package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.webcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "webcamTesting" , group = "TeleOp")
public class aprilTagDetection extends OpMode {
    webcam aprilTagWebcam = new webcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificID(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);

    }
}
