package org.firstinspires.ftc.teamcode.trainee;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "servoOpMode", group = "TeleOp")
public class servoOpmode extends OpMode {
    servoTrial servo = new servoTrial();
    @Override
    public void init() {
        servo.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            servo.setServoPos(-0.25);
        } else if (gamepad1.right_bumper) {
            servo.setServoPos(0.25);
        } else {
            servo.setServoPos(0);
        }



    }
}
