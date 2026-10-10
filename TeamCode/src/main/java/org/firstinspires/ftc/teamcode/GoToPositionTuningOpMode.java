package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/*
 * Live tuning for GoToPositionController. Edits the controller's static constants from the
 * gamepad, runs a repeatable test move, and reports overshoot / drift / settle time.
 *
 * Controls (gamepad1):
 *   dpad up / down      select parameter
 *   dpad right / left   increase / decrease selected parameter (hold R1 for 5x step)
 *   Square              cycle test: FORWARD, STRAFE, DIAGONAL, TURN
 *   Triangle            cycle test distance: 24, 48, 72 inches
 *   Cross               run test (moves alternate out / back so the robot stays in place)
 *   Circle              abort test
 *   sticks              drive manually to reposition while no test is running; abort a running test
 *
 * Parameters are shown under their GoToPositionController constant names.
 * HEADING_TOLERANCE_RADIANS and TURN_DEADBAND_RADIANS are shown and edited in degrees.
 *
 * Values are NOT saved. Copy the "Copy into GoToPositionController" lines into the code when done.
 *
 * Tuning order: MIN_TURN_POWER, MIN_DRIVE_POWER, TURN_DEADBAND_RADIANS, HEADING_TOLERANCE_RADIANS,
 * POSITION_TOLERANCE_INCHES, KP_TURN, MAX_TURN_POWER, KP_DRIVE, STRAFE_MULTIPLIER, MAX_DRIVE_POWER.
 */

@TeleOp(name="SC2627 Tuning: GoToPosition", group="Tuning")
public class GoToPositionTuningOpMode extends LinearOpMode {

    private enum TestType { FORWARD, STRAFE, DIAGONAL, TURN }

    private enum TestState { IDLE, RUNNING, SETTLING }

    // Max time the controller gets to reach the target before the test is marked TIMEOUT
    private static final double TEST_TIMEOUT_SECONDS = 6.0;
    // Keep measuring after the controller stops, to catch coasting past the target
    private static final double SETTLE_SECONDS = 0.75;
    private static final double MANUAL_DRIVE_SPEED = 0.5;
    private static final double STICK_ABORT_THRESHOLD = 0.20;
    private static final double[] TEST_DISTANCES = {24.0, 48.0, 72.0};

    private static class Param {
        final String name;
        final double step;
        final DoubleSupplier getter;
        final DoubleConsumer setter;
        final double min;

        Param(String name, double step, double min, DoubleSupplier getter, DoubleConsumer setter) {
            this.name = name;
            this.step = step;
            this.min = min;
            this.getter = getter;
            this.setter = setter;
        }
    }

    private final Param[] params = {
            new Param("KP_DRIVE", 0.005, 0.0,
                    () -> GoToPositionController.KP_DRIVE,
                    v -> GoToPositionController.KP_DRIVE = v),
            new Param("KP_TURN", 0.05, 0.0,
                    () -> GoToPositionController.KP_TURN,
                    v -> GoToPositionController.KP_TURN = v),
            new Param("STRAFE_MULTIPLIER", 0.05, 0.5,
                    () -> GoToPositionController.STRAFE_MULTIPLIER,
                    v -> GoToPositionController.STRAFE_MULTIPLIER = v),
            new Param("MAX_DRIVE_POWER", 0.05, 0.05,
                    () -> GoToPositionController.MAX_DRIVE_POWER,
                    v -> GoToPositionController.MAX_DRIVE_POWER = Math.min(1.0, v)),
            new Param("MIN_DRIVE_POWER", 0.01, 0.0,
                    () -> GoToPositionController.MIN_DRIVE_POWER,
                    v -> GoToPositionController.MIN_DRIVE_POWER = v),
            new Param("MAX_TURN_POWER", 0.05, 0.05,
                    () -> GoToPositionController.MAX_TURN_POWER,
                    v -> GoToPositionController.MAX_TURN_POWER = Math.min(1.0, v)),
            new Param("MIN_TURN_POWER", 0.01, 0.0,
                    () -> GoToPositionController.MIN_TURN_POWER,
                    v -> GoToPositionController.MIN_TURN_POWER = v),
            new Param("POSITION_TOLERANCE_INCHES", 0.25, 0.25,
                    () -> GoToPositionController.POSITION_TOLERANCE_INCHES,
                    v -> GoToPositionController.POSITION_TOLERANCE_INCHES = v),
            new Param("HEADING_TOLERANCE_RADIANS (deg)", 0.5, 0.5,
                    () -> Math.toDegrees(GoToPositionController.HEADING_TOLERANCE_RADIANS),
                    v -> GoToPositionController.HEADING_TOLERANCE_RADIANS = Math.toRadians(v)),
            new Param("TURN_DEADBAND_RADIANS (deg)", 0.5, 0.0,
                    () -> Math.toDegrees(GoToPositionController.TURN_DEADBAND_RADIANS),
                    v -> GoToPositionController.TURN_DEADBAND_RADIANS = Math.toRadians(v)),
    };

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private GoToPositionController controller;

    private final ElapsedTime testTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    private int selectedParam = 0;
    private TestType testType = TestType.FORWARD;
    private int distanceIndex = 1;
    private TestState testState = TestState.IDLE;
    // Flips every run so tests go out and back
    private double direction = 1.0;

    // Test geometry (field frame)
    private double startX, startY, startHeading;
    private double targetX, targetY, targetHeading;
    private double moveDirX, moveDirY; // unit vector of the move, zero for TURN
    private double testDistance;

    // Measurements for the current / last test
    private boolean reachedTarget;
    private boolean aborted;
    private double timeToReach;
    private double maxOvershoot;      // inches past the target (linear) or degrees past (turn)
    private double maxCrossTrack;     // signed inches off the straight line at the worst point, + = left of travel
    private double maxHeadingDrift;   // degrees away from start heading (linear tests)
    private double finalPositionError;
    private double finalHeadingErrorDeg;
    private String lastResult = "none yet";

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Same directions as SC2627Base
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake so stops are repeatable between runs
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Start pose doesn't matter here: every test is relative to where the robot is when A is pressed
        controller = new GoToPositionController(frontLeftDrive, backLeftDrive, frontRightDrive,
                backRightDrive, pinpoint, telemetry, GoToPositionController.START_POS.BLUE, true);

        telemetry.addLine("GoToPosition tuning ready. Give the robot room to move "
                + (int) TEST_DISTANCES[TEST_DISTANCES.length - 1] + "\" in every direction.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            controller.odometry();

            boolean sticksActive = Math.abs(gamepad1.left_stick_y) > STICK_ABORT_THRESHOLD
                    || Math.abs(gamepad1.left_stick_x) > STICK_ABORT_THRESHOLD
                    || Math.abs(gamepad1.right_stick_x) > STICK_ABORT_THRESHOLD;

            handleParamInput();

            // Read every button edge each loop so a press in one state doesn't fire later in another
            boolean xPressed = gamepad1.xWasPressed();
            boolean yPressed = gamepad1.yWasPressed();
            boolean aPressed = gamepad1.aWasPressed();
            boolean bPressed = gamepad1.bWasPressed();

            switch (testState) {

                case IDLE:
                    if (xPressed) {
                        testType = TestType.values()[(testType.ordinal() + 1) % TestType.values().length];
                    }
                    if (yPressed) {
                        distanceIndex = (distanceIndex + 1) % TEST_DISTANCES.length;
                    }
                    if (aPressed) {
                        startTest();
                    } else {
                        manualDrive();
                    }
                    break;

                case RUNNING:
                    if (bPressed || sticksActive) {
                        aborted = true;
                        controller.stopMotors();
                        finishTest();
                        break;
                    }
                    measure();
                    if (controller.autoAssistDrive()) {
                        reachedTarget = true;
                        timeToReach = testTimer.seconds();
                        beginSettling();
                    } else if (testTimer.seconds() > TEST_TIMEOUT_SECONDS) {
                        timeToReach = testTimer.seconds();
                        beginSettling();
                    }
                    break;

                case SETTLING:
                    measure();
                    if (settleTimer.seconds() > SETTLE_SECONDS) {
                        finishTest();
                    }
                    break;
            }

            addTelemetry();
            telemetry.update();
        }

        controller.stopMotors();
    }

    private void handleParamInput() {
        if (gamepad1.dpadUpWasPressed()) {
            selectedParam = (selectedParam + params.length - 1) % params.length;
        }
        if (gamepad1.dpadDownWasPressed()) {
            selectedParam = (selectedParam + 1) % params.length;
        }

        double multiplier = gamepad1.right_bumper ? 5.0 : 1.0;
        Param p = params[selectedParam];
        if (gamepad1.dpadRightWasPressed()) {
            p.setter.accept(p.getter.getAsDouble() + p.step * multiplier);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            p.setter.accept(Math.max(p.min, p.getter.getAsDouble() - p.step * multiplier));
        }
    }

    private void manualDrive() {
        double axial   = (Math.abs(gamepad1.left_stick_y)  > 0.08) ? -gamepad1.left_stick_y : 0.0;
        double lateral = (Math.abs(gamepad1.left_stick_x)  > 0.08) ?  gamepad1.left_stick_x : 0.0;
        double yaw     = (Math.abs(gamepad1.right_stick_x) > 0.08) ?  gamepad1.right_stick_x : 0.0;

        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl * MANUAL_DRIVE_SPEED);
        frontRightDrive.setPower(fr * MANUAL_DRIVE_SPEED);
        backLeftDrive.setPower(bl * MANUAL_DRIVE_SPEED);
        backRightDrive.setPower(br * MANUAL_DRIVE_SPEED);
    }

    private void startTest() {
        startX = controller.getX();
        startY = controller.getY();
        startHeading = controller.getHeading();
        testDistance = TEST_DISTANCES[distanceIndex];

        if (testType == TestType.TURN) {
            moveDirX = 0.0;
            moveDirY = 0.0;
            targetX = startX;
            targetY = startY;
            targetHeading = GoToPositionController.normalizeAngle(startHeading + direction * Math.PI / 2);
        } else {
            // Angle of the move relative to the robot's heading (CCW positive, so right is negative)
            double relativeAngle;
            if (testType == TestType.FORWARD) {
                relativeAngle = 0.0;
            } else if (testType == TestType.STRAFE) {
                relativeAngle = -Math.PI / 2;
            } else {
                relativeAngle = -Math.PI / 4;
            }
            double moveAngle = startHeading + relativeAngle;
            moveDirX = direction * Math.cos(moveAngle);
            moveDirY = direction * Math.sin(moveAngle);
            targetX = startX + moveDirX * testDistance;
            targetY = startY + moveDirY * testDistance;
            targetHeading = startHeading;
        }

        controller.setTargetPositionAndPath(targetX, targetY, targetHeading);

        reachedTarget = false;
        aborted = false;
        timeToReach = 0.0;
        maxOvershoot = 0.0;
        maxCrossTrack = 0.0;
        maxHeadingDrift = 0.0;

        testTimer.reset();
        testState = TestState.RUNNING;
    }

    private void beginSettling() {
        controller.stopMotors();
        settleTimer.reset();
        testState = TestState.SETTLING;
    }

    private void measure() {
        double x = controller.getX();
        double y = controller.getY();
        double heading = controller.getHeading();

        if (testType == TestType.TURN) {
            // Progress in the commanded turn direction, in degrees
            double progressDeg = Math.toDegrees(
                    GoToPositionController.normalizeAngle(heading - startHeading)) * direction;
            maxOvershoot = Math.max(maxOvershoot, progressDeg - 90.0);
        } else {
            double dx = x - startX;
            double dy = y - startY;
            double along = dx * moveDirX + dy * moveDirY;
            double cross = -dx * moveDirY + dy * moveDirX;
            maxOvershoot = Math.max(maxOvershoot, along - testDistance);
            if (Math.abs(cross) > Math.abs(maxCrossTrack)) {
                maxCrossTrack = cross;
            }
            maxHeadingDrift = Math.max(maxHeadingDrift,
                    Math.abs(Math.toDegrees(GoToPositionController.normalizeAngle(heading - startHeading))));
        }

        finalPositionError = Math.hypot(targetX - x, targetY - y);
        finalHeadingErrorDeg = Math.toDegrees(GoToPositionController.normalizeAngle(targetHeading - heading));
    }

    private void finishTest() {
        String outcome = aborted ? "ABORTED" : (reachedTarget ? "REACHED" : "TIMEOUT");
        if (testType == TestType.TURN) {
            lastResult = String.format("%s %s %s in %.2fs | overshoot %.1f deg | final err %.1f\" %.1f deg",
                    testType, direction > 0 ? "CCW" : "CW", outcome, timeToReach,
                    maxOvershoot, finalPositionError, finalHeadingErrorDeg);
        } else {
            lastResult = String.format("%s %.0f\" %s %s in %.2fs | overshoot %.1f\" | off-line %+.1f\" (+ = left)"
                            + " | heading drift %.1f deg | final err %.1f\" %.1f deg",
                    testType, testDistance, direction > 0 ? "out" : "back", outcome, timeToReach,
                    maxOvershoot, maxCrossTrack, maxHeadingDrift,
                    finalPositionError, finalHeadingErrorDeg);
        }
        // Only flip direction if the robot actually went out, so the next run brings it back
        if (!aborted) {
            direction = -direction;
        }
        testState = TestState.IDLE;
    }

    private void addTelemetry() {
        telemetry.addData("Test", "%s  %.0f\"  next: %s   [Square: test, Triangle: distance, Cross: run, Circle: abort]",
                testType, TEST_DISTANCES[distanceIndex], direction > 0 ? "out" : "back");
        telemetry.addData("State", testState);
        telemetry.addData("Last result", lastResult);
        telemetry.addData("Pose (X, Y, Heading)", "(%.1f, %.1f, %.1f deg)",
                controller.getX(), controller.getY(), Math.toDegrees(controller.getHeading()));

        telemetry.addLine();
        telemetry.addLine("Parameters  [dpad up/down: select, left/right: change, hold R1: 5x]");
        for (int i = 0; i < params.length; i++) {
            Param p = params[i];
            telemetry.addLine(String.format("%s %-32s %.3f", i == selectedParam ? ">>" : "  ",
                    p.name, p.getter.getAsDouble()));
        }

        telemetry.addLine();
        telemetry.addLine("Copy into GoToPositionController:");
        telemetry.addLine(String.format("KP_DRIVE = %.3f;", GoToPositionController.KP_DRIVE));
        telemetry.addLine(String.format("KP_TURN = %.3f;", GoToPositionController.KP_TURN));
        telemetry.addLine(String.format("STRAFE_MULTIPLIER = %.3f;", GoToPositionController.STRAFE_MULTIPLIER));
        telemetry.addLine(String.format("MAX_DRIVE_POWER = %.3f;", GoToPositionController.MAX_DRIVE_POWER));
        telemetry.addLine(String.format("MIN_DRIVE_POWER = %.3f;", GoToPositionController.MIN_DRIVE_POWER));
        telemetry.addLine(String.format("MAX_TURN_POWER = %.3f;", GoToPositionController.MAX_TURN_POWER));
        telemetry.addLine(String.format("MIN_TURN_POWER = %.3f;", GoToPositionController.MIN_TURN_POWER));
        telemetry.addLine(String.format("POSITION_TOLERANCE_INCHES = %.3f;",
                GoToPositionController.POSITION_TOLERANCE_INCHES));
        telemetry.addLine(String.format("HEADING_TOLERANCE_RADIANS = Math.toRadians(%.1f);",
                Math.toDegrees(GoToPositionController.HEADING_TOLERANCE_RADIANS)));
        telemetry.addLine(String.format("TURN_DEADBAND_RADIANS = Math.toRadians(%.1f);",
                Math.toDegrees(GoToPositionController.TURN_DEADBAND_RADIANS)));
    }
}
