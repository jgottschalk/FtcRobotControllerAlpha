package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GoBildaDirectFlywheel {
    public static final double TICKS_PER_REV = 28.0;
    public static final double MAX_FREE_RPM = 6000.0;
    public static final double MAX_FREE_TICKS_PER_SEC = (MAX_FREE_RPM / 60.0) * TICKS_PER_REV; // 2800.0

    // Direct Drive 1:1 baseline parameters
    protected double kP = 0.0022;
    protected double kI = 0.0;
    protected double kD = 0.00008;
    protected double kS = 0.025;
    protected double kV = 1.0 / MAX_FREE_TICKS_PER_SEC; // ~0.000357
    protected double toleranceRpm = 75.0;

    protected final DcMotorEx motor;
    protected final VoltageSensor batterySensor;
    protected final ElapsedTime timer = new ElapsedTime();

    protected double targetVelocityTicks = 0.0;
    protected double targetRpm = 0.0;
    protected double lastError = 0.0;
    protected double integral = 0.0;
    protected double lastTime = 0.0;
    protected double lastPower = 0.0;
    protected double burstImpulse = 0.0;

    protected double[] velocityBuffer = new double[3];
    protected int bufferIndex = 0;
    protected double filteredVelocityTicks = 0.0;

    public GoBildaDirectFlywheel(DcMotorEx motor, VoltageSensor batterySensor) {
        this.motor = motor;
        this.batterySensor = batterySensor;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        timer.reset();
        lastTime = timer.seconds();
    }

    public void setTargetRPM(double rpm) {
        this.targetRpm = rpm;
        this.targetVelocityTicks = (rpm / 60.0) * TICKS_PER_REV;
        if (rpm == 0.0) {
            integral = 0.0;
            lastError = 0.0;
        }
    }

    public void applyShotImpulse(double boostMagnitude) {
        this.burstImpulse = boostMagnitude;
    }

    public void update() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        if (dt <= 0.0 || dt > 0.2) {
            dt = 0.02;
        }

        // Rolling average filter for 28 PPR encoder noise
        double rawTicks = motor.getVelocity();
        velocityBuffer[bufferIndex] = rawTicks;
        bufferIndex = (bufferIndex + 1) % velocityBuffer.length;

        double sum = 0.0;
        for (double v : velocityBuffer) sum += v;
        filteredVelocityTicks = sum / velocityBuffer.length;

        if (targetRpm <= 0.0) {
            motor.setPower(0.0);
            burstImpulse = 0.0;
            lastPower = 0.0;
            return;
        }

        // PID
        double error = targetVelocityTicks - filteredVelocityTicks;
        double proportional = kP * error;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double feedbackPower = proportional + (kI * integral) + (kD * derivative);

        // Feedforward: kS + kV + impulse boost
        double feedforwardPower = (kS * Math.signum(targetVelocityTicks))
                + (kV * targetVelocityTicks)
                + burstImpulse;

        // Battery voltage sag normalization (12.0V baseline)
        double busVoltage = Math.max(batterySensor.getVoltage(), 8.0);
        double totalOutput = (feedbackPower + feedforwardPower) * (12.0 / busVoltage);

        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));
        motor.setPower(totalOutput);
        lastPower = totalOutput;

        burstImpulse *= 0.35;
    }

    public boolean isAtSpeed() {
        return Math.abs(getCurrentRPM() - targetRpm) < toleranceRpm;
    }

    public double getCurrentRPM() {
        return (filteredVelocityTicks / TICKS_PER_REV) * 60.0;
    }

    public double getTargetRPM() {
        return targetRpm;
    }

    public double getLastPower() {
        return lastPower;
    }
}