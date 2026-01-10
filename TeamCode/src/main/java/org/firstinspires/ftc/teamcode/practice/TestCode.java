package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Getting Velocity", group = "TeleOp")
public class TestCode extends OpMode {
    private DcMotorEx shooter;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
        shooter.setPower(1.0);

        double velocityTicksPerSec = shooter.getVelocity();
        double omega = shooter.getVelocity(AngleUnit.RADIANS);

        double rpm = (omega * 60) / (2 * Math.PI);

        telemetry.addData("Velocity (ticks/sec)", velocityTicksPerSec);
        telemetry.addData("Omega (rad/sec)", omega);
        telemetry.addData("RPM (rev/min)", rpm);
    }
}

