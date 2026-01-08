package org.firstinspires.ftc.teamcode.trainee;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flywheel PIDF Tuner" )
public class FlywheelTuningOpMode extends OpMode {

    FlywheelPIDF flywheel;

    double targetRPM = 4000;

    // Tuning step sizes
    double kFStep = 0.000005;
    double kPStep = 0.00001;
    double rpmStep = 100;

    @Override
    public void init() {
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotor.Direction.REVERSE);

        flywheel = new FlywheelPIDF(shooter);
    }

    @Override
    public void loop() {

        // --- Live tuning controls ---
        if (gamepad1.b) flywheel.kF += kFStep;
        if (gamepad1.x) flywheel.kF -= kFStep;

        if (gamepad1.y) flywheel.kP += kPStep;
        if (gamepad1.a) flywheel.kP -= kPStep;

        if (gamepad1.dpad_up) targetRPM += rpmStep;
        if (gamepad1.dpad_down) targetRPM -= rpmStep;

        targetRPM = Math.max(0, Math.min(6000, targetRPM));

        // --- PIDF update ---
        flywheel.setTargetRPM(targetRPM);
        flywheel.update();

        // --- Telemetry ---
        telemetry.addLine("=== Flywheel PIDF Tuning ===");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", flywheel.currentRPM);
        telemetry.addData("Error", targetRPM - flywheel.currentRPM);
        telemetry.addData("Motor Power", flywheel.lastPower);

        telemetry.addLine("--- Constants ---");
        telemetry.addData("kF", flywheel.kF);
        telemetry.addData("kP", flywheel.kP);
        telemetry.addData("kI", flywheel.kI);
        telemetry.addData("kD", flywheel.kD);

        telemetry.update();
    }
}
