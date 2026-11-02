package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Flywheel control in volts:  V = kS + kV * targetRpm + kP * (targetRpm - rpm)
 * then divided by measured battery voltage so behavior is consistent as the battery sags.
 *
 * Call update() once per loop.
 */
public class FlywheelController {
    private static final long VOLTAGE_READ_PERIOD_NS = 100_000_000L; // 100 ms

    private final DcMotorEx motor;
    private final VoltageSensor voltageSensor;
    private final FlywheelParams p;

    private boolean spinning = false;
    private boolean openLoop = false;
    private double openLoopVolts = 0;

    private double targetRpm;
    private double effectiveTargetRpm;
    private double rpm = 0;
    private double batteryV = 12.0;
    private double commandedVolts = 0;
    private boolean saturated = false;

    private long lastVoltageReadNs = 0;
    private long lastUpdateNs = 0;
    private long inWindowSinceNs = -1;
    private double saturatedMsAccum = 0;

    public FlywheelController(DcMotorEx flywheelMotor, VoltageSensor vs, FlywheelParams params) {
        this.p = params;
        motor = flywheelMotor;

        //this is crucial
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Brake at zero power so the flywheel stops quickly when spinning is turned off.
        // While spinning, the controller always commands some power, so this doesn't affect speed control.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = vs;
        targetRpm = params.targetRpm;
        batteryV = Math.max(voltageSensor.getVoltage(), 1.0);
    }

    // ---------------- Commands ----------------

    public void setTargetRpm(double rpm) {
        this.targetRpm = Math.max(0, rpm);
    }

    public void setSpinning(boolean on) {
        spinning = on;
        openLoop = false;
        inWindowSinceNs = -1;
    }

    /** Bypass control and apply a fixed voltage (used by the kS/kV sweep). */
    public void setOpenLoopVolts(double volts) {
        openLoop = true;
        spinning = false;
        openLoopVolts = volts;
        inWindowSinceNs = -1;
    }

    public void stop() {
        spinning = false;
        openLoop = false;
        inWindowSinceNs = -1;
        motor.setPower(0);
    }

    public void update() {
        long now = System.nanoTime();
        double dtMs = lastUpdateNs == 0 ? 0 : (now - lastUpdateNs) / 1e6;
        lastUpdateNs = now;

        if (now - lastVoltageReadNs > VOLTAGE_READ_PERIOD_NS) {
            double v = voltageSensor.getVoltage();
            if (v > 1.0) batteryV = v;
            lastVoltageReadNs = now;
        }

        double raw = motor.getVelocity() * 60.0 / p.ticksPerRev;
        rpm += p.velFilterAlpha * (raw - rpm);

        effectiveTargetRpm = Math.min(targetRpm, maxTargetRpm());

        double volts;
        if (openLoop) {
            volts = openLoopVolts;
        } else if (spinning) {
            double ff = effectiveTargetRpm > 0 ? p.kS + p.kV * effectiveTargetRpm : 0;
            volts = ff + p.kP * (effectiveTargetRpm - rpm);
        } else {
            volts = 0;
        }

        saturated = volts >= batteryV;
        if (saturated) saturatedMsAccum += dtMs;
        commandedVolts = Math.max(0, Math.min(batteryV, volts));
        motor.setPower(commandedVolts / batteryV);

        // Ready-window bookkeeping
        if (spinning && effectiveTargetRpm > 0
                && Math.abs(effectiveTargetRpm - rpm) <= p.toleranceRpm) {
            if (inWindowSinceNs < 0) inWindowSinceNs = now;
        } else {
            inWindowSinceNs = -1;
        }
    }

    /**
     * Highest RPM we allow at the present battery voltage while keeping headroom volts in reserve
     * for recovery after a ball loads the wheel.
     */
    public double maxTargetRpm() {
        if (p.kV <= 1e-9) return Double.MAX_VALUE;
        return Math.max(0, (batteryV * (1.0 - p.backEmfHeadroom) - p.kS) / p.kV);
    }

    /** True if the requested target is being limited by headroom. */
    public boolean isCappedByHeadroom() {
        return targetRpm > maxTargetRpm();
    }

    /** True when velocity has been inside the tolerance window for at least readyDwellMs. */
    public boolean isReady() {
        return inWindowSinceNs >= 0
                && (System.nanoTime() - inWindowSinceNs) / 1e6 >= p.readyDwellMs;
    }

    public boolean isSpinning() { return spinning; }
    public double getRpm() { return rpm; }
    public double getTargetRpm() { return targetRpm; }
    public double getEffectiveTargetRpm() { return effectiveTargetRpm; }
    public double getBatteryVolts() { return batteryV; }
    public double getCommandedVolts() { return commandedVolts; }
    public boolean isSaturated() { return saturated; }
    public double getSaturatedMs() { return saturatedMsAccum; }
    public void resetSaturatedMs() { saturatedMsAccum = 0; }
}
