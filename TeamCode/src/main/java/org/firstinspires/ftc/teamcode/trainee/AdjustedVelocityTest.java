package org.firstinspires.ftc.teamcode.trainee;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "AdjustedVelocityTest")
public class AdjustedVelocityTest extends OpMode {

    private static final double TICKS_PER_REV = 28;
    private DcMotorEx shooter;
    static final double shooter_radius = 0.0762;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.getZeroPowerBehavior();
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void loop() {

        // Run shooter at full power
        shooter.setPower(0.1);

        // Get encoder velocity (ticks per second)
        double ticksPerSecond = shooter.getVelocity();

        // Convert ticks/sec → revolutions/sec
        double revsPerSecond = ticksPerSecond / TICKS_PER_REV;

        // Convert to RPM
        double rpm = revsPerSecond * 60.0;

        // Telemetry output
        telemetry.addData("Ticks/sec", ticksPerSecond);
        telemetry.addData("RPM", rpm);
        telemetry.update();
    }
}

//    @Override
//    public void loop() {
//        // max rpm is 6000 tested!, 6000 rpm = 100 rps = 200π rad/sec (angular vel) = 13.716π m/s (lin vel)
//
//        flywheel.setVelocity(200 * Math.PI, AngleUnit.RADIANS);
//
//        //flywheel.getVelocity() // returns the velocity in ticks per sec
//
//        double angularVelocity = flywheel.getVelocity(AngleUnit.RADIANS); // Calculate the angular velocity in radians per second
//
//        double revolutionsPerMin = (angularVelocity / (2 * Math.PI)) * 60;
//
//        double tangentialVelocity = angularVelocity * FLYWHEEL_RADIUS; // Calculate the tangential velocity in meters per second
//
//        telemetry.addData("RPM (rev/min)", revolutionsPerMin );
//        telemetry.addData("Angular Velocity (rad/sec)", angularVelocity);
//        telemetry.addData("Tangential Velocity (m/s)", tangentialVelocity);
//        telemetry.update();
//    }
//}


/*
To find kf for feedforward ( kF ≈ power / velocityInTicks)
1. Make the the flywheel runs at full power
2. measure velocity in ticks per sec
 */

