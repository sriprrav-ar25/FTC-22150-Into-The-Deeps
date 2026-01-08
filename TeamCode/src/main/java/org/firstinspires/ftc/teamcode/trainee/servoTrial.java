package org.firstinspires.ftc.teamcode.trainee;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoTrial {

    private CRServo servoRot1, servoRot2;

    public void init(HardwareMap hardwareMap) {
        servoRot1 = hardwareMap.get(CRServo.class, "servo1");
        servoRot2 = hardwareMap.get(CRServo.class, "servo2");

        servoRot1.setDirection(CRServo.Direction.REVERSE);
        servoRot2.setDirection(CRServo.Direction.FORWARD);


    }

    public void setServoPos(double angle) {
        servoRot1.setPower(angle);
        servoRot2.setPower(angle);
    }

}
