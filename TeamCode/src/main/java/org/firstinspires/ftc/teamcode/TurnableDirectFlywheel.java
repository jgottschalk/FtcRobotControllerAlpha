package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class TurnableDirectFlywheel {

    // Encoder counts per revolution (adjust if your motor/gearing differs)
    public static double TICKS_PER_REV = 28.0;

    // Runtime-tunable gains controlled by the OpMode
    public static double DASH_KP = 0.0022;
    public static double DASH_KI = 0.0;
    public static double DASH_KD = 0.00008;
    public static double DASH_KS = 0.025;
    public static double DASH_KV = 1.0 / 2800.0;
    public static double DASH_TOLERANCE_RPM = 75.0;
    public static double SHOT_IMPULSE_POWER = 0.15;

    private final DcMotorEx motor;
    private final VoltageSensor batterySensor;

    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastPower = 0.0;
    private long lastTimeNs = 0;

    public TurnableDirectFlywheel(DcMotorEx motor, VoltageSensor batterySensor) {
        this.motor = motor;
        this.batterySensor = batterySensor;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = Math.max(0.0, rpm);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public double getMotorPower() {
        return lastPower;
    }

    public void triggerConfiguredImpulse() {
        // Simulates shot drop by briefly cutting power
        lastPower = Math.max(0.0, lastPower - SHOT_IMPULSE_POWER);
        motor.setPower(lastPower);
    }

    public void update() {
        long nowNs = System.nanoTime();
        if (lastTimeNs == 0) {
            lastTimeNs = nowNs;
            return;
        }

        double dt = (nowNs - lastTimeNs) / 1.0e9;
        lastTimeNs = nowNs;

        if (dt <= 0.0 || dt > 0.5) {
            return;
        }

        // Calculate velocity in RPM from motor ticks/sec
        double ticksPerSec = motor.getVelocity();
        currentRPM = (ticksPerSec / TICKS_PER_REV) * 60.0;

        if (targetRPM <= 0.0) {
            motor.setPower(0.0);
            lastPower = 0.0;
            integralSum = 0.0;
            lastError = 0.0;
            return;
        }

        double error = targetRPM - currentRPM;

        // PID calculations
        double pTerm = DASH_KP * error;

        integralSum += error * dt;
        double iTerm = DASH_KI * integralSum;

        double dTerm = DASH_KD * ((error - lastError) / dt);
        lastError = error;

        // Feedforward calculations: kV * RPM + kS friction offset
        double ffTerm = (DASH_KV * targetRPM) + (targetRPM > 0 ? DASH_KS : 0.0);

        // Nominal battery voltage compensation
        double batteryVoltage = batterySensor.getVoltage();
        double voltageCompensation = (batteryVoltage > 8.0) ? (12.0 / batteryVoltage) : 1.0;

        double totalCommand = (ffTerm + pTerm + iTerm + dTerm) * voltageCompensation;

        // Clamp power between 0.0 and 1.0 (unidirectional flywheel)
        lastPower = Math.max(0.0, Math.min(1.0, totalCommand));
        motor.setPower(lastPower);
    }
}