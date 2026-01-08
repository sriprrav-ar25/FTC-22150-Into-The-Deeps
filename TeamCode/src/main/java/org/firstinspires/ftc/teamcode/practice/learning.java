package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Getting Velocity", group = "TeleOp")
public class learning extends OpMode {
    private DcMotorEx shooter;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // Use encoder as sensor only
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        //shooter.setPower(0);
    }

    @Override
    public void loop() {
        shooter.setPower(1.0);

        double omega = shooter.getVelocity(AngleUnit.RADIANS);
        telemetry.addData("ticks", shooter.getCurrentPosition());
        telemetry.addData("rad/sec", omega);
        telemetry.update();




    }

//    @Override
//    public void stop() {
//        shooter.setPower(0);
//    }
}
