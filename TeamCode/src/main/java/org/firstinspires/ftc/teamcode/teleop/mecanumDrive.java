package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.mecanumDrivee;

@TeleOp(name = "mecanumDrive", group = "TeleOp")
public class mecanumDrive extends OpMode {
    mecanumDrivee drive = new mecanumDrivee();
    double forward, strafe, rotate;


    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.fieldOrient(forward, strafe, rotate);
    }
}
