package org.firstinspires.ftc.teamcode.trainee;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class motorTrial {
    private DcMotor testMotor;
    private Telemetry telemetry;
    private double ticksPerRev;

    public void init(HardwareMap hardwareMap) {
        testMotor = hardwareMap.get(DcMotor.class, "motorTest");

        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ticksPerRev = testMotor.getMotorType().getTicksPerRev();
    }
    public void setPower(double power) {
        testMotor.setPower(power);
    }

    public double getMotorRev() {
        return testMotor.getCurrentPosition() / ticksPerRev;
    }


}
