package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Flywheel + timed feeder, managed by your OpMode. This class knows nothing about gamepads or
 * OpMode lifecycle: the OpMode maps inputs to the methods below and calls update() every loop.
 *
 * Firing = wait until the flywheel is inside the ready window, then run the feeder for a fixed
 * time (long enough to push all four spheres through), then stop the feeder.
 *
 * Typical use:
 *   init:  flywheel = new FlywheelOperate(hardwareMap);
 *   loop:  map buttons -> flywheel.toggleSpin() / fire() / cancel() ...
 *          flywheel.update();
 *          flywheel.addTelemetry(telemetry);
 *   stop:  flywheel.stop();
 *
 */
public class FlywheelOperate {

    public enum State { IDLE, WAIT_READY, FEEDING }

    //TODO: move to speeds.java
    public double feedMs = 2000;
    /** Abort a gated fire if the wheel doesn't become ready in this long. */
    public double readyTimeoutMs = 2000;

    private FlywheelController fw = null;

    private CRServo windMill = null;

    private State state = State.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private boolean gateOnReady = true;
    private double rpmAtFeedStart = 0;
    private double minRpmDuringFeed = 0;
    private String message = "";

    /** Loads the parameters saved by FlywheelTuner. */

    public FlywheelOperate(DcMotorEx motor, VoltageSensor vs, CRServo windmill) {
        FlywheelParams params = new FlywheelParams();
        this.fw = new FlywheelController(motor, vs, params);
        windMill = windmill;
    }

    // ---------------- Commands ----------------

    public void setSpinning(boolean on) {
        fw.setSpinning(on);
        if (!on) cancel("Flywheel stopped");
    }

    public void toggleSpin() { setSpinning(!fw.isSpinning()); }

    public void setTargetRpm(double rpm) { fw.setTargetRpm(rpm); }

    public void adjustTargetRpm(double delta) { fw.setTargetRpm(fw.getTargetRpm() + delta); }

    public void adjustFeedMs(double delta) { feedMs = Math.max(50, feedMs + delta); }

    /** Wait for the ready window, then run the feeder for feedMs. Ignored unless spinning and idle. */
    public void fire() { startFire(true); }

    /** Run the feeder immediately without waiting for ready (for testing). */
    public void fireUngated() { startFire(false); }

    /** Stop the feeder and return to idle. Flywheel keeps spinning. */
    public void cancel() { cancel("Cancelled"); }

    /** Stop everything. Call from the OpMode's stop / end of loop. */
    public void stop() {
        stopWindmill();
        state = State.IDLE;
        fw.stop();
    }

    public void stopWindmill() {
        windMill.setPower(0.0);
    }

    public void startWindmill() {
        windMill.setPower(0.5);
    }

    // ---------------- Update ----------------

    /** Call once per OpMode loop. */
    public void update() {
        fw.update();
        if (!fw.isSpinning() && state != State.IDLE) cancel("Flywheel stopped");

        switch (state) {
            case IDLE:
                break;

            case WAIT_READY:
                if (!gateOnReady || fw.isReady()) {
                    rpmAtFeedStart = fw.getRpm();
                    minRpmDuringFeed = fw.getRpm();
                    startWindmill();
                    message = "Feeding";
                    enter(State.FEEDING);
                } else if (stateTimer.milliseconds() > readyTimeoutMs) {
                    cancel("Aborted: never ready (raise headroom / lower target?)");
                }
                break;

            case FEEDING:
                minRpmDuringFeed = Math.min(minRpmDuringFeed, fw.getRpm());
                if (stateTimer.milliseconds() >= feedMs) {
                    stopWindmill();
                    message = String.format("Done. RPM %.0f -> min %.0f (drop %.0f)",
                            rpmAtFeedStart, minRpmDuringFeed, rpmAtFeedStart - minRpmDuringFeed);
                    enter(State.IDLE);
                }
                break;
        }
    }

    // ---------------- Queries ----------------

    public State getState() { return state; }
    public boolean isBusy() { return state != State.IDLE; }
    public boolean isSpinning() { return fw.isSpinning(); }
    public boolean isReady() { return fw.isReady(); }
    public double getRpm() { return fw.getRpm(); }
    public double getTargetRpm() { return fw.getTargetRpm(); }
    public String getMessage() { return message; }
    /** Direct access to the controller for advanced use. */
    public FlywheelController getController() { return fw; }

    /** Adds status lines to telemetry. The OpMode is responsible for calling telemetry.update(). */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("State", "%s  (%s)", state, message);
        telemetry.addData("Feed time", "%.0f ms", feedMs);
        telemetry.addData("Spinning", fw.isSpinning());
        telemetry.addData("Target / Effective", "%.0f / %.0f RPM%s", fw.getTargetRpm(),
                fw.getEffectiveTargetRpm(), fw.isCappedByHeadroom() ? "  CAPPED BY HEADROOM" : "");
        telemetry.addData("RPM", "%.0f  (err %.0f)", fw.getRpm(), fw.getEffectiveTargetRpm() - fw.getRpm());
        telemetry.addData("Ready", fw.isReady());
        telemetry.addData("Battery", "%.2f V", fw.getBatteryVolts());
        telemetry.addData("Command", "%.2f V%s", fw.getCommandedVolts(), fw.isSaturated() ? "  SATURATED" : "");
    }

    // ---------------- Internals ----------------

    private void startFire(boolean gated) {
        if (!fw.isSpinning() || state != State.IDLE) return;
        gateOnReady = gated;
        message = gated ? "Waiting for ready..." : "Firing (ungated)...";
        enter(State.WAIT_READY);
    }

    private void cancel(String why) {
        stopWindmill();
        message = why;
        enter(State.IDLE);
    }

    private void enter(State s) {
        state = s;
        stateTimer.reset();
    }

}
