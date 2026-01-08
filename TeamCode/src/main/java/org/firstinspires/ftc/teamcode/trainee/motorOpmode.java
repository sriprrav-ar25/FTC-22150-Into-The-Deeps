package org.firstinspires.ftc.teamcode.trainee;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "motorTestOpMode", group = "TeleOp")
public class motorOpmode extends OpMode {
    motorTrial motorTest = new motorTrial();

    @Override
    public void init() {
        motorTest.init(hardwareMap);
    }


    @Override
    public void loop() {
        motorTest.setPower(0.25);
        telemetry.addData("Motor Rev", motorTest.getMotorRev());
        telemetry.update();




    }
}
