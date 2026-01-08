package org.firstinspires.ftc.teamcode.trainee;

import com.qualcomm.robotcore.hardware.DcMotor;
public class FlywheelPIDF {
    private DcMotor motor;

    // ---------------- PIDF Constants ----------------
    // Start with F + P only
    public double kF = 1.0 / 6000.0;   // ≈ 1 / maxRPM (6000)
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;

    // ---------------- State Variables ----------------
    public double targetRPM = 0.0;
    public double currentRPM = 0.0;
    public double lastPower = 0.0;

    private double integral = 0.0;
    private double lastError = 0.0;

    // Encoder tracking
    private double lastTicks = 0.0;
    private double lastTime = 0.0;

    // Encoder constants
    private final int TICKS_PER_REV = 28; // goBILDA motor encoder

    // ---------------- Constructor ----------------
    public FlywheelPIDF(DcMotor motor) {
        this.motor = motor;
        this.lastTime = System.nanoTime();
        this.lastTicks = motor.getCurrentPosition();
    }

    // ---------------- Public Methods ----------------

    /** Set desired flywheel speed */
    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0, rpm);
    }

    /** Call this EVERY loop() */
    public void update() {

        // --- Time step ---
        double now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        if (dt <= 0) return;

        // --- Measure RPM ---
        currentRPM = calculateRPM(dt);

        // --- PID terms ---
        double error = targetRPM - currentRPM;

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        // --- PIDF output ---
        double power =
                (kF * targetRPM) +
                        (kP * error) +
                        (kI * integral) +
                        (kD * derivative);

        // --- Safety clamp ---
        power = Math.max(0.0, Math.min(1.0, power));

        motor.setPower(power);
        lastPower = power;
    }

    // ---------------- Helper Methods ----------------

    /** Calculate flywheel RPM from encoder */
    private double calculateRPM(double dt) {
        double ticks = motor.getCurrentPosition();
        double deltaTicks = ticks - lastTicks;
        lastTicks = ticks;

        double revolutions = deltaTicks / TICKS_PER_REV;
        return (revolutions / dt) * 60.0;
    }
}
