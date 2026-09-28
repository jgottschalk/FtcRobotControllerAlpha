package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.speeds;

/**
 * How to tune:
 *   1. Run the sweep with X button (A). Outcome set kS and kV.
 *   2. Spin up with SQUARE button (X). Outcome - tune kP with Left/Right stick until the wheel settles quickly without oscillating
 *   3. Launch a ball with TRIANGLE button (Y). Outcome - If "saturated during recovery" is > 0 ms, the
 *      controller ran out of voltage: raise EMF headroom (lowers max target) with d-pad
 *      up/down or lower target RPM.
 *   4. Run noise measurement - Spin up with SQUARE button (X), then measure with CIRCLE button (B). Outcome - tolerance must be
 *      comfortably above 3*sigma or the ready flag will flicker; make it as small as you
 *      can tolerate for shot consistency. Use d-pad left and d-pad right to adjust towards
 *      recommended tolerance.
 *
 *  *   A                   Run kS/kV sweep (flywheel spins freely; keep balls and feeder clear!)
 *  *   X                   Toggle closed-loop spin at target RPM
 *  *   Left/Right bumper   Target RPM -/+ 100
 *  *   D-pad up/down       Back-EMF headroom +/- 0.01
 *  *   D-pad right/left    Ready tolerance +/- 5 RPM
 *  *   Left/Right stick    kP -/+ 0.0005 (V per RPM)
 *  *   B                   Measure steady-state velocity noise (2 s) -> suggests tolerance
 *  *   Y                   Arm shot; then fire a ball; records droop, recovery time and
 *  *                       whether the controller hit the battery ceiling while recovering
 *  *   share               Save all parameters (loaded by FlywheelOperate)
 *
 */
@TeleOp(name = "Flywheel Tuner", group = "Linear OpMode")
public class FlywheelTuner extends LinearOpMode {

    // Sweep settings
    private static final int SWEEP_STEPS = 6;
    private static final double SWEEP_SETTLE_MS = 1000;
    private static final double SWEEP_SAMPLE_MS = 600;
    private static final double SWEEP_MAX_FRACTION_OF_VBAT = 0.85;
    private static final double SWEEP_MIN_FRACTION_OF_VBAT = 0.20;

    // Shot capture settings
    private static final double SHOT_DETECT_DROP_RPM = 60;
    private static final double SHOT_TIMEOUT_MS = 3000;

    private FlywheelParams params;
    private FlywheelController fw;

    // Results / status
    private String status = "Ready";
    private double noiseSigma = Double.NaN;
    private double lastDroop = Double.NaN;
    private double lastRecoveryMs = Double.NaN;
    private double lastSaturatedMs = Double.NaN;
    private boolean shotArmed = false;
    private boolean shotCapturing = false;
    private double shotMinRpm;
    private final ElapsedTime shotTimer = new ElapsedTime();

    private DcMotorEx launcher = null;

    private CRServo windMill = null;

    private VoltageSensor voltageSensor = null;

    private boolean paramSave = false;

    @Override
    public void runOpMode() {

        windMill = hardwareMap.get(CRServo.class, "windmill");
        windMill.setDirection(CRServo.Direction.FORWARD);
        windMill.setPower(speeds.STOP_SPEED);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        params = FlywheelParams.load();
        fw = new FlywheelController(launcher, voltageSensor, params);

        boolean prevA = false, prevB = false, prevX = false, prevY = false;
        boolean prevLB = false, prevRB = false, prevUp = false, prevDown = false;
        boolean prevLeft = false, prevRight = false, prevLS = false, prevRS = false;
        boolean prevBack = false;

        telemetry.addLine("Flywheel Tuner ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // ---- Inputs (edge-triggered) ----
            if (edge(gamepad1.left_bumper, prevLB)) params.targetRpm = Math.max(0, params.targetRpm - 100);
            if (edge(gamepad1.right_bumper, prevRB)) params.targetRpm += 100;
            if (edge(gamepad1.dpad_up, prevUp))
                params.backEmfHeadroom = clamp(params.backEmfHeadroom + 0.01, 0, 0.6);
            if (edge(gamepad1.dpad_down, prevDown))
                params.backEmfHeadroom = clamp(params.backEmfHeadroom - 0.01, 0, 0.6);
            if (edge(gamepad1.dpad_right, prevRight)) params.toleranceRpm += 5;
            if (edge(gamepad1.dpad_left, prevLeft)) params.toleranceRpm = Math.max(5, params.toleranceRpm - 5);
            if (edge(gamepad1.left_stick_button, prevLS)) params.kP = Math.max(0, params.kP - 0.0005);
            if (edge(gamepad1.right_stick_button, prevRS)) params.kP += 0.0005;

            if (edge(gamepad1.x, prevX)) {
                fw.setSpinning(!fw.isSpinning());
                status = fw.isSpinning() ? "Spinning" : "Stopped";
            }
            if (edge(gamepad1.y, prevY) && fw.isSpinning()) {
                shotArmed = true;
                shotCapturing = false;
                status = "Shot armed - fire a ball";
            }
            if (edge(gamepad1.share, prevBack)) {
                params.save();
                status = "Saved parameters";
                paramSave = true;
                telemetry.addData("ticksPerRev", params.ticksPerRev);
                telemetry.addData("kS", params.kS);
                telemetry.addData("kV", params.kV);
                telemetry.addData("kP", params.kP);
                telemetry.addData("backEmfHeadroom", params.backEmfHeadroom);
                telemetry.addData("toleranceRpm", params.toleranceRpm);
                telemetry.addData("readyDwellMs", params.readyDwellMs);
                telemetry.addData("velFilterAlpha", params.velFilterAlpha);
                telemetry.addData("targetRpm", params.targetRpm);
            }

            boolean doSweep = edge(gamepad1.a, prevA);
            boolean doNoise = edge(gamepad1.b, prevB);

            prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x; prevY = gamepad1.y;
            prevLB = gamepad1.left_bumper; prevRB = gamepad1.right_bumper;
            prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
            prevLeft = gamepad1.dpad_left; prevRight = gamepad1.dpad_right;
            prevLS = gamepad1.left_stick_button; prevRS = gamepad1.right_stick_button;
            prevBack = gamepad1.back;

            fw.setTargetRpm(params.targetRpm);

            if (doSweep) runSweep();
            if (doNoise) measureNoise();

            fw.update();
            updateShotCapture();

            if (paramSave) {
                paramSave = false;
            } else {
                showTelemetry();
            }
        }
        fw.stop();
    }

    // ------------------------------------------------------------------------------------
    // kS / kV sweep: apply fixed voltages, record steady-state RPM, fit V = kS + kV * rpm
    // ------------------------------------------------------------------------------------
    private void runSweep() {
        fw.setSpinning(false);
        double vbat = fw.getBatteryVolts();
        double vLo = vbat * SWEEP_MIN_FRACTION_OF_VBAT;
        double vHi = vbat * SWEEP_MAX_FRACTION_OF_VBAT;

        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        int n = 0;

        for (int i = 0; i < SWEEP_STEPS && opModeIsActive(); i++) {
            double volts = vLo + (vHi - vLo) * i / (SWEEP_STEPS - 1);
            fw.setOpenLoopVolts(volts);
            status = String.format("Sweep %d/%d @ %.2f V", i + 1, SWEEP_STEPS, volts);

            // Settle
            ElapsedTime t = new ElapsedTime();
            while (opModeIsActive() && t.milliseconds() < SWEEP_SETTLE_MS) {
                fw.update();
                showTelemetry();
            }
            // Sample
            t.reset();
            double sum = 0;
            int count = 0;
            while (opModeIsActive() && t.milliseconds() < SWEEP_SAMPLE_MS) {
                fw.update();
                sum += fw.getRpm();
                count++;
                showTelemetry();
            }
            if (count == 0) continue;
            double rpm = sum / count;
            if (rpm < 50) continue; // still below stiction, unusable for the fit

            sx += rpm; sy += volts; sxx += rpm * rpm; sxy += rpm * volts; n++;
        }
        fw.stop();

        if (n >= 3) {
            double denom = n * sxx - sx * sx;
            if (Math.abs(denom) > 1e-9) {
                double kV = (n * sxy - sx * sy) / denom;
                double kS = (sy - kV * sx) / n;
                if (kV > 0) {
                    params.kV = kV;
                    params.kS = Math.max(0, kS);
                    status = String.format("Sweep done: kS=%.3f V, kV=%.5f V/RPM", params.kS, params.kV);
                    return;
                }
            }
        }
        status = "Sweep failed (too few usable points). Check encoder direction/ticksPerRev.";
    }

    // ------------------------------------------------------------------------------------
    // Noise measurement: std dev of filtered RPM while holding target
    // ------------------------------------------------------------------------------------
    private void measureNoise() {
        if (!fw.isSpinning()) {
            status = "Spin up (X) and let it settle before measuring noise";
            return;
        }
        ElapsedTime t = new ElapsedTime();
        double sum = 0, sumSq = 0;
        int n = 0;
        while (opModeIsActive() && t.milliseconds() < 2000) {
            fw.update();
            double r = fw.getRpm();
            sum += r; sumSq += r * r; n++;
            status = "Measuring noise...";
            showTelemetry();
        }
        if (n > 10) {
            double mean = sum / n;
            noiseSigma = Math.sqrt(Math.max(0, sumSq / n - mean * mean));
            status = String.format("Noise sigma = %.1f RPM", noiseSigma);
        }
    }

    // ------------------------------------------------------------------------------------
    // Shot capture: detect droop after a ball is fired, time recovery into the ready window
    // ------------------------------------------------------------------------------------
    private void updateShotCapture() {
        if (shotArmed && !shotCapturing
                && fw.getRpm() < fw.getEffectiveTargetRpm() - SHOT_DETECT_DROP_RPM) {
            shotCapturing = true;
            shotTimer.reset();
            shotMinRpm = fw.getRpm();
            fw.resetSaturatedMs();
            status = "Shot detected, recovering...";
        }
        if (shotCapturing) {
            shotMinRpm = Math.min(shotMinRpm, fw.getRpm());
            boolean recovered = fw.isReady();
            boolean timedOut = shotTimer.milliseconds() > SHOT_TIMEOUT_MS;
            windMill.setPower(0.5);
            if (recovered || timedOut) {
                lastDroop = fw.getEffectiveTargetRpm() - shotMinRpm;
                lastRecoveryMs = timedOut ? Double.NaN : shotTimer.milliseconds();
                lastSaturatedMs = fw.getSaturatedMs();
                shotCapturing = false;
                shotArmed = false;
                status = timedOut ? "Shot capture timed out (never re-entered window)"
                                  : "Shot captured";
            }
        } else {
            windMill.setPower(0.0);
        }
    }

    // ------------------------------------------------------------------------------------
    private void showTelemetry() {
        double suggestedTol = Double.isNaN(noiseSigma) ? Double.NaN
                : Math.ceil(Math.max(3 * noiseSigma, 0.02 * params.targetRpm) / 5.0) * 5.0;

        telemetry.addData("Status", status);
        telemetry.addData("Battery (V)", "%.2f", fw.getBatteryVolts());
        telemetry.addData("Target / Effective RPM", "%.0f / %.0f",
                fw.getTargetRpm(), fw.getEffectiveTargetRpm());
        telemetry.addData("RPM", "%.0f", fw.getRpm());
        telemetry.addData("Command (V)", "%.2f%s", fw.getCommandedVolts(), fw.isSaturated() ? "  SATURATED" : "");
        telemetry.addData("Ready", fw.isReady());
        telemetry.addData("kS (V)", "%.3f", params.kS);
        telemetry.addData("kV (V/RPM)", "%.5f", params.kV);
        telemetry.addData("kP (V/RPM)", "%.4f", params.kP);
        telemetry.addData("Back-EMF headroom", "%.0f%%", params.backEmfHeadroom * 100);
        telemetry.addData("Max target @ this battery", "%.0f RPM%s", fw.maxTargetRpm(),
                fw.isCappedByHeadroom() ? "  (TARGET CAPPED)" : "");
        telemetry.addData("Ready tolerance", "+/- %.0f RPM for %.0f ms",
                params.toleranceRpm, params.readyDwellMs);
        telemetry.addData("Noise sigma", Double.isNaN(noiseSigma) ? "n/a" : String.format("%.1f RPM", noiseSigma));
        telemetry.addData("Suggested tolerance", Double.isNaN(suggestedTol) ? "n/a"
                : String.format("%.0f RPM (max of 3*sigma, 2%% of target)", suggestedTol));
        telemetry.addData("Last shot droop", Double.isNaN(lastDroop) ? "n/a" : String.format("%.0f RPM", lastDroop));
        telemetry.addData("Last recovery time", Double.isNaN(lastRecoveryMs) ? "n/a" : String.format("%.0f ms", lastRecoveryMs));
        telemetry.addData("Saturated during recovery", Double.isNaN(lastSaturatedMs) ? "n/a"
                : String.format("%.0f ms%s", lastSaturatedMs, lastSaturatedMs > 0 ? "  -> raise headroom" : ""));
        telemetry.update();
    }

    private static boolean edge(boolean now, boolean prev) { return now && !prev; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
