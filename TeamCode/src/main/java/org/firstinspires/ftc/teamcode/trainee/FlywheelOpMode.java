//package org.firstinspires.ftc.teamcode.trainee;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//public class FlywheelOpMode extends OpMode {
//    FlywheelPIDF flywheel;
//    @Override
//    public void init() {
//        DcMotor shooter = hardwareMap.get(DcMotor.class, "flywheel");
//        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        flywheel = new FlywheelPIDF(shooter);
//    }
//
//    @Override
//    public void loop() {
//        double targetRPM = calculateRPMFromAprilTag();
//        flywheel.setTargetRPM(targetRPM);
//        flywheel.update();
//
//        telemetry.addData("Target RPM", targetRPM);
//        telemetry.addData("Current RPM", flywheel.currentRPM);
//        telemetry.update();
//    }
//
//}
