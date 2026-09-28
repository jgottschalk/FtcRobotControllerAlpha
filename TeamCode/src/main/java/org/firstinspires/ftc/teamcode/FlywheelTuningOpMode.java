package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp(name = "Flywheel Tuning (Gamepad)", group = "Tuning")
public class FlywheelTuningOpMode extends LinearOpMode {

    // Tuning parameters
    private double targetRPM = 5000.0;
    private double kP = 0.0022;
    private double kD = 0.00008;
    private double kV = 1.0 / 2800.0;
    private double kS = 0.025;

    private boolean isRunning = false;

    // Gamepad debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    @Override
    public void runOpMode() {
        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        VoltageSensor batterySensor = hardwareMap.voltageSensor.iterator().next();

        TurnableDirectFlywheel flywheel = new TurnableDirectFlywheel(flywheelMotor, batterySensor);

        telemetry.addLine("=== SYSTEM READY ===");
        telemetry.addLine("A: Run | B: Stop");
        telemetry.addLine("D-Pad Up/Down: kP (+/- 0.0002)");
        telemetry.addLine("D-Pad Right/Left: kD (+/- 0.00002)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Run state toggles
            if (gamepad1.a) isRunning = true;
            if (gamepad1.b) isRunning = false;

            // Live gain adjustments via D-pad
            if (gamepad1.dpad_up && !lastDpadUp) kP += 0.0002;
            if (gamepad1.dpad_down && !lastDpadDown) kP = Math.max(0.0, kP - 0.0002);
            if (gamepad1.dpad_right && !lastDpadRight) kD += 0.00002;
            if (gamepad1.dpad_left && !lastDpadLeft) kD = Math.max(0.0, kD - 0.00002);

            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadRight = gamepad1.dpad_right;
            lastDpadLeft = gamepad1.dpad_left;

            // Push updated gains
            TurnableDirectFlywheel.DASH_KP = kP;
            TurnableDirectFlywheel.DASH_KD = kD;
            TurnableDirectFlywheel.DASH_KS = kS;
            TurnableDirectFlywheel.DASH_KV = kV;

            flywheel.setTargetRPM(isRunning ? targetRPM : 0.0);
            flywheel.update();

            // Driver Station Display
            telemetry.addData("STATE", isRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", "%.1f", flywheel.getCurrentRPM());
            telemetry.addData("Error RPM", "%.1f", (targetRPM - flywheel.getCurrentRPM()));
            telemetry.addLine("--- Active Gains ---");
            telemetry.addData("kP", "%.5f", kP);
            telemetry.addData("kD", "%.6f", kD);
            telemetry.addData("kV", "%.6f", kV);
            telemetry.addData("Battery (V)", "%.2f", batterySensor.getVoltage());
            telemetry.update();
        }

        flywheelMotor.setPower(0.0);
    }
}