package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Basic Run")
public class runMotor extends OpMode {
    DcMotor shooter;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotor.class, "motorTest");

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    @Override
    public void loop() {

        shooter.setPower(1.0);
    }
}
