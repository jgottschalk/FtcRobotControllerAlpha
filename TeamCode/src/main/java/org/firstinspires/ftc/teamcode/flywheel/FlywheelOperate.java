package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.speeds;

import java.util.Arrays;

/**
 * Flywheel + timed feeder, managed by your OpMode. This class knows nothing about gamepads
 * or OpMode lifecycle. The OpMode maps inputs to the methods below and calls update() every loop.
 *
 * Firing = wait until the flywheel is inside the ready window, then run the feeder for a fixed
 * time (long enough to push all four spheres through), then stop the feeder:
 *   init:  flywheel = new FlywheelOperate(hardwareMap);
 *   loop:  map buttons -> flywheel.toggleSpin() / fire() / cancel() ...
 *          flywheel.update();
 *          flywheel.addTelemetry(telemetry);
 *   stop:  flywheel.stop();
 *
 */
public class FlywheelOperate {

    private enum FlywheelState { IDLE, LAUNCHING, FEEDING, REALIGNING}

    private FlywheelController fw = null;
    private CRServo windMill = null;

    private FlywheelState state = FlywheelState.IDLE;
    private double rpmAtFeedStart = 0;
    private double minRpmDuringFeed = 0;
    private String message = "";
    private final double[] shotExitRpm = new double[]{-1, -1, -1, -1};
    private int shotCount = 0;

    protected boolean windMillCancelled = false;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime windmillTimer = new ElapsedTime();

    private RevColorSensorV3 color = null;
    private String colorState = "";

    private double feedingMinRpm = Double.MAX_VALUE;
    private boolean droopDetected = false;

    private static final double SHOT_DETECT_DROP_RPM = 50.0; // Confirms ball entered
    private static final double REBOUND_THRESHOLD_RPM = 25.0; // Confirms ball left



    public FlywheelOperate(DcMotorEx motor, VoltageSensor vs, CRServo windmill, RevColorSensorV3 colorSensor) {
        FlywheelParams params = new FlywheelParams();
        this.fw = new FlywheelController(motor, vs, params);
        windMill = windmill;
        color = colorSensor;
    }

    public void setSpinning(boolean on) {
        fw.setSpinning(on);
    }

    public void toggleSpin() {
        setSpinning(!fw.isSpinning());
    }

    public void stop() {
        state = FlywheelState.IDLE;
        fw.stop();
        shotCount = 0;
    }

    public void stopWindmill() {
        windMillCancelled = true;
        windMill.setPower(speeds.STOP_SPEED);
    }

    public void startWindmill() { windMill.setPower(1.0); }

    public boolean isWindmillSpinning() {
        return windMill.getPower() > 0.0;
    }

    private FlywheelState getState() { return state; }
    public boolean isSpinning() { return fw.isSpinning(); }
    public boolean isReady() { return fw.isReady(); }
    public double getRpm() { return fw.getRpm(); }
    public double getTargetRpm() { return fw.getTargetRpm(); }
    public String getMessage() { return message; }
    public FlywheelController getController() { return fw; }

    public boolean idle() {return getState() == FlywheelState.IDLE; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("State", "%s  (%s)", getState(), getMessage());
        telemetry.addData("Spinning", isSpinning());
        telemetry.addData("Target / Effective", "%.0f / %.0f RPM%s", getTargetRpm(),
                fw.getEffectiveTargetRpm(), fw.isCappedByHeadroom() ? "  CAPPED BY HEADROOM" : "");
        telemetry.addData("RPM", "%.0f  (err %.0f)", getRpm(), fw.getEffectiveTargetRpm() - fw.getRpm());
        telemetry.addData("Ready", isReady());
        telemetry.addData("Battery", "%.2f V", fw.getBatteryVolts());
        telemetry.addData("Command", "%.2f V%s", fw.getCommandedVolts(), fw.isSaturated() ? "  SATURATED" : "");
        telemetry.addData("Shots (1-4)", "%.0f | %.0f | %.0f | %.0f RPM",
                shotExitRpm[0], shotExitRpm[1], shotExitRpm[2], shotExitRpm[3]);
        telemetry.addData("color", colorState);
    }

    private void enter(FlywheelState s) {
        state = s;
        stateTimer.reset();
    }

    public void launch() {
        shotCount = 0;
        Arrays.fill(shotExitRpm, -1.0);
        droopDetected = false;
        feedingMinRpm = Double.MAX_VALUE;
        enter(FlywheelState.LAUNCHING);
    }

    public void update() {

        fw.update();

        switch (state) {
            case IDLE:
                //no-op
                break;

            case LAUNCHING:
                if (fw.isReady()) {
                    rpmAtFeedStart = fw.getRpm();
                    minRpmDuringFeed = fw.getRpm();
                    startWindmill();
                    windmillTimer.reset();
                    message = "Feeding";
                    enter(FlywheelState.FEEDING);
                }

                break;

            case FEEDING:

                minRpmDuringFeed = Math.min(minRpmDuringFeed, fw.getRpm());
                double currentRpm = fw.getRpm();

                // 1. Detect entry into the wheel
                if (!droopDetected) {
                    if (currentRpm < fw.getEffectiveTargetRpm() - SHOT_DETECT_DROP_RPM) {
                        droopDetected = true;
                        feedingMinRpm = currentRpm;
                    }
                } else {
                    // 2. Track the bottom of the dip
                    if (currentRpm < feedingMinRpm) {
                        feedingMinRpm = currentRpm;
                    }
                    // 3. LAUNCH EVENT TRIGGER: Wheel has rebounded by 25 RPM above trough
                    else if (currentRpm > feedingMinRpm + REBOUND_THRESHOLD_RPM) {
                        // Ball has cleared the wheel!
                        stopWindmill();

                        // Record shot velocity telemetry
                        shotExitRpm[shotCount % 4] = feedingMinRpm;
                        shotCount++;

                        // Clean up state and gate on the next shot
                        droopDetected = false;
                        feedingMinRpm = Double.MAX_VALUE;

                        if (shotCount >= 4) {
                            enter(FlywheelState.REALIGNING);
                        } else {
                            // Waits for isReady() before restarting windmill!
                            enter(FlywheelState.LAUNCHING);
                        }
                    }
                }

                if (stateTimer.seconds() >= speeds.WINDMILL_TIME_SECONDS) {
                    message = String.format("Done. RPM %.0f -> min %.0f (drop %.0f)",
                            rpmAtFeedStart, minRpmDuringFeed, rpmAtFeedStart - minRpmDuringFeed);
                    enter(FlywheelState.REALIGNING);
                }
                break;

            case REALIGNING:

                if (!isWindmillSpinning()) startWindmill();

                final double buffer = 0.5;

                boolean stopWindmill = (windmillTimer.seconds() > buffer + speeds.WINDMILL_TIME_SECONDS) &&
                        //(color.blue() > 43) &&
                        //(color.blue() < 51) &&
                        (color.alpha() <= 33) && //PMS: 45 - 50; scrimmage: 33
                        (color.getDistance(DistanceUnit.CM) >= 5.5);
                        //(color.getDistance(DistanceUnit.CM) < 7.0);

                colorState =
                                "red: " + color.red() +
                                ", green: " + color.green() +
                                ", blue: " + color.blue() +
                                ", alpha: " + color.alpha() +
                                ", distance: " + color.getDistance(DistanceUnit.CM);

                if (stopWindmill || windMillCancelled ) {
                    stopWindmill();
                    windMillCancelled = false;
                    enter(FlywheelState.IDLE);
                }

                break;

        }

        }

    //public void setTargetRpm(double rpm) { fw.setTargetRpm(rpm); }
    //public void adjustTargetRpm(double delta) { fw.setTargetRpm(fw.getTargetRpm() + delta); }
    //public void adjustFeedMs(double delta) { feedMs = Math.max(50, feedMs + delta); }
    //public void fireUngated() { startFire(false); } Run the feeder immediately without waiting for ready (for testing).
    //public void cancel() { cancel("Cancelled"); } //Stop the feeder and return to idle. Flywheel keeps spinning.

}