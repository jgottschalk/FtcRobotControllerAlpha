
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.flywheel.FlywheelOperate;

import java.util.ArrayList;
import java.util.List;

/*
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * gamepad X: SQUARE BUTTON
 * gamepad A: X BUTTON
 * gamepad Y: TRIANGLE BUTTON
 * gamepad B: CIRCLE BUTTON
*/

@TeleOp(name="SC2627 TeleOp: Base for Testing", group="Linear OpMode")
//@Disabled
public class SC2627Base extends LinearOpMode {

    protected enum xTakeMode {
        INTAKE,
        POSITIONING,
        OUTTAKE
    }

    protected enum DriveMode {
        MANUAL,
        AUTO_ASSIST
    }

    //timers
    protected final ElapsedTime m_runtime = new ElapsedTime();
    double lastCycleEndTime = 0.0;

    //power draws
    protected DcMotor frontLeftDrive = null;
    protected DcMotor backLeftDrive = null;
    protected DcMotor frontRightDrive = null;
    protected DcMotor backRightDrive = null;
    protected DcMotorEx launcher = null;
    protected DcMotorEx intake = null;
    protected CRServo leftFeeder = null;
    protected CRServo rightFeeder = null;
    protected CRServo windMill = null;
    protected RevColorSensorV3 color = null;
    protected VoltageSensor voltageSensor = null;
    protected GoBildaPinpointDriver pinpoint = null;

    //controllers
    protected FlywheelOperate fwOperate = null;
    protected GoToPositionController positionController = null;

    //modes
    protected DriveMode m_driveMode = DriveMode.MANUAL;
    protected xTakeMode m_xTakeMode = xTakeMode.INTAKE;

    //buttons
    boolean lastYButtonState = false;
    boolean lastAButtonState = false;
    boolean lastBButtonState = false;
    boolean lastShareButtonState = false;

    // Manual drive ramp: a stick command builds from 0 to full over DRIVE_RAMP_SECONDS, even if the
    // stick is pushed all the way at once. Easing off, letting go and reversing take effect at once
    // (a reverse drops to 0 first, then builds up again in the new direction).
    public static double DRIVE_RAMP_SECONDS = 0.5;

    // Manual drive top speeds. Forward/back and turning use speeds.DEFAULT_SPEED;
    // strafing (left stick sideways) has its own top speed. Raise or lower STRAFE_SPEED (max 1.0).

    private double rampedAxial = 0.0;
    private double rampedLateral = 0.0;
    private double rampedYaw = 0.0;
    private final ElapsedTime m_rampTimer = new ElapsedTime();

    GoToPositionController.START_POS startPos = null;

    public void initRobot(GoToPositionController.START_POS ps, boolean firstPreference) {

        startPos = ps;

        m_xTakeMode = xTakeMode.POSITIONING;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        intake = hardwareMap.get(DcMotorEx.class, "Intook");

        windMill = hardwareMap.get(CRServo.class, "windmill");

        color = hardwareMap.get(RevColorSensorV3.class, "color");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        fwOperate = new FlywheelOperate(launcher, voltageSensor, windMill, color);

        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake on zero power. Must match GoToPositionTuningOpMode so tuned gains behave the same here.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setPower(speeds.STOP_SPEED);
        rightFeeder.setPower(speeds.STOP_SPEED);

        windMill.setDirection(CRServo.Direction.FORWARD);
        windMill.setPower(speeds.STOP_SPEED);

        leftFeeder.setDirection(CRServo.Direction.FORWARD);
        rightFeeder.setDirection(CRServo.Direction.REVERSE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        positionController = new GoToPositionController(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, pinpoint, telemetry, startPos, firstPreference);

        // TeleOp continues from where autonomous left the robot, if autonomous saved a pose recently.
        // Autonomous always starts from the start position.
        boolean isAutonomous = getClass().isAnnotationPresent(Autonomous.class);
        if (!isAutonomous && PoseStorage.hasPose()) {
            positionController.setPose(PoseStorage.getX(), PoseStorage.getY(), PoseStorage.getHeading());
            // Show what the Pinpoint itself now reports, not just what was saved
            positionController.odometry();
            telemetry.addData("Start pose", "from autonomous (%.1f, %.1f, %.1f deg)",
                    positionController.getX(), positionController.getY(),
                    Math.toDegrees(positionController.getHeading()));
        } else {
            telemetry.addLine("Start pose: start position");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.setAutoClear(true);
        telemetry.update();

        m_runtime.reset();

    }

    // Autonomous segment timing: each endSegment() records how long that numbered segment took
    private final ElapsedTime m_segmentTimer = new ElapsedTime();
    private final List<String> m_segmentTimes = new ArrayList<>();
    private double m_segmentTotalSeconds = 0.0;

    /** Call right after waitForStart() in an autonomous, before the first segment. */
    protected void startSegmentTiming() {
        m_segmentTimes.clear();
        m_segmentTotalSeconds = 0.0;
        m_segmentTimer.reset();
    }

    /** Call at the end of each segment: records its time (also written to the log, tag SC2627). */
    protected void endSegment(String name) {
        double seconds = m_segmentTimer.seconds();
        m_segmentTimer.reset();
        m_segmentTotalSeconds += seconds;
        String line = String.format("%s: %.2f s", name, seconds);
        m_segmentTimes.add(line);
        RobotLog.ii("SC2627", "segment " + line);
    }

    /** Adds every recorded segment time, and the total, to the telemetry. */
    protected void addSegmentTelemetry() {
        telemetry.addLine("Segment times");
        for (String line : m_segmentTimes) {
            telemetry.addLine(line);
        }
        telemetry.addLine(String.format("Total: %.2f s", m_segmentTotalSeconds));
    }

    protected void activateIntakeMode() {
        intake.setPower(speeds.FULL_SPEED);
        leftFeeder.setPower(speeds.FULL_SPEED);
        rightFeeder.setPower(speeds.FULL_SPEED);
    }

    protected void deactivateIntakeMode() {
        intake.setPower(speeds.STOP_SPEED);
        leftFeeder.setPower(speeds.STOP_SPEED);
        rightFeeder.setPower(speeds.STOP_SPEED);
    }

    protected void activateOuttakeMode () {
        fwOperate.toggleSpin();
    }

    protected void deactivateOuttakeMode () {
        fwOperate.toggleSpin();
    }

    /**
     * Switches straight to a mode, setting the intake and flywheel to match it:
     * INTAKE = intake on, flywheel off; POSITIONING = both off; OUTTAKE = intake off, flywheel spinning.
     * Leaving OUTTAKE also cancels any launch in progress.
     */
    protected void setXTakeMode(xTakeMode mode) {
        switch (mode) {
            case INTAKE:
                fwOperate.stop();
                fwOperate.stopWindmill();
                activateIntakeMode();
                break;
            case POSITIONING:
                fwOperate.stop();
                fwOperate.stopWindmill();
                deactivateIntakeMode();
                break;
            case OUTTAKE:
                deactivateIntakeMode();
                fwOperate.setSpinning(true);
                break;
        }
        m_xTakeMode = mode;
    }

    @Override
    public void runOpMode() {

        //default to blue for testing purposes
        initRobot(GoToPositionController.START_POS.BLUE, true);
        waitForStart();

        while (opModeIsActive()) {

            positionController.odometry();

            boolean currentYButtonState = gamepad1.y;
            boolean toggleYButtonState;
            if (currentYButtonState && !lastYButtonState) {
                toggleYButtonState = true;
            } else {
                toggleYButtonState = false;
            }
            lastYButtonState = currentYButtonState;

            switch (m_xTakeMode) {

                case INTAKE:
                    if (toggleYButtonState) {
                        deactivateIntakeMode();
                        m_xTakeMode = xTakeMode.POSITIONING;
                    }
                    break;

                case POSITIONING:
                    if (toggleYButtonState) {
                        activateOuttakeMode();
                        m_xTakeMode = xTakeMode.OUTTAKE;
                    }
                    break;

                case OUTTAKE:
                    if (toggleYButtonState) {
                        deactivateOuttakeMode();
                        activateIntakeMode();
                        m_xTakeMode = xTakeMode.INTAKE;
                    }
                    break;

            }

            // D-pad: jump straight to a mode (up = launch, left = intake, right = neither)
            boolean dpadUpPressed = gamepad1.dpadUpWasPressed();
            boolean dpadLeftPressed = gamepad1.dpadLeftWasPressed();
            boolean dpadRightPressed = gamepad1.dpadRightWasPressed();
            if (dpadUpPressed) {
                setXTakeMode(xTakeMode.POSITIONING);
            } else if (dpadLeftPressed) {
                setXTakeMode(xTakeMode.INTAKE);
            } else if (dpadRightPressed) {
                setXTakeMode(xTakeMode.OUTTAKE);
            }

            double max;

            double axial   = (Math.abs(gamepad1.left_stick_y)  > 0.08) ? -gamepad1.left_stick_y : 0.0;
            double lateral = (Math.abs(gamepad1.left_stick_x)  > 0.08) ?  gamepad1.left_stick_x : 0.0;
            double yaw     = (Math.abs(gamepad1.right_stick_x) > 0.08) ?  gamepad1.right_stick_x : 0.0;

            // Ramp the stick commands so power builds up gradually instead of all at once
            double rampDt = Math.min(m_rampTimer.seconds(), 0.1);
            m_rampTimer.reset();
            double rampStep = rampDt / DRIVE_RAMP_SECONDS;
            if (m_driveMode == DriveMode.MANUAL) {
                rampedAxial = ramp(rampedAxial, axial, rampStep);
                rampedLateral = ramp(rampedLateral, lateral, rampStep);
                rampedYaw = ramp(rampedYaw, yaw, rampStep);
            } else {
                // Auto-assist is driving: start from 0 again when manual driving resumes
                rampedAxial = 0.0;
                rampedLateral = 0.0;
                rampedYaw = 0.0;
            }
            axial = rampedAxial * speeds.DEFAULT_SPEED;
            lateral = rampedLateral * speeds.STRAFE_SPEED;
            yaw = rampedYaw * speeds.DEFAULT_SPEED;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize so no wheel exceeds the top speed (the higher of DEFAULT_SPEED and STRAFE_SPEED),
            // keeping the ratio between wheels so the robot still moves in the requested direction.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            double wheelCap = Math.max(speeds.DEFAULT_SPEED, speeds.STRAFE_SPEED);
            if (max > wheelCap) {
                double scale = wheelCap / max;
                frontLeftPower *= scale;
                frontRightPower *= scale;
                backLeftPower *= scale;
                backRightPower *= scale;
            }

            modulateIntake();

            boolean launchNow = (gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased());

            if (launchNow && (m_xTakeMode == xTakeMode.OUTTAKE)) {
                fwOperate.launch();
            } else if (launchNow && (m_xTakeMode == xTakeMode.INTAKE || m_xTakeMode == xTakeMode.POSITIONING)) {
                gamepad1.rumbleBlips(3);
            }

            fwOperate.update();

            fwOperate.addTelemetry(telemetry);

            switch (m_driveMode) {

                case AUTO_ASSIST:

                    boolean driverIntervention = Math.abs(gamepad1.left_stick_y) > 0.20
                            || Math.abs(gamepad1.left_stick_x) > 0.20
                            || Math.abs(gamepad1.right_stick_x) > 0.20;

                    boolean destinationReached = positionController.autoAssistDrive();

                    if (destinationReached) {
                        positionController.stopMotors();
                        gamepad1.rumbleBlips(1);
                        m_driveMode = DriveMode.MANUAL;
                    } else if (driverIntervention) {
                        m_driveMode = DriveMode.MANUAL;
                        positionController.stopMotors();
                    }

                    break;

                case MANUAL:

                    boolean currentAButtonState = gamepad1.a;
                    boolean currentBButtonState = gamepad1.b;

                    if ((currentAButtonState && !lastAButtonState)
                    || (currentBButtonState && !lastBButtonState)) {

                        GoToPositionController.GOAL_POS position;
                        if (currentAButtonState && !lastAButtonState) {
                            position = GoToPositionController.GOAL_POS.NEAR;
                        } else {
                            position = GoToPositionController.GOAL_POS.FAR;
                        }

                        positionController.goScore(position);
                        m_driveMode = DriveMode.AUTO_ASSIST;
                    }

                    lastAButtonState = currentAButtonState;
                    lastBButtonState = currentBButtonState;

                    if (m_driveMode == DriveMode.MANUAL) {

                            frontLeftDrive.setPower(frontLeftPower);
                            frontRightDrive.setPower(frontRightPower);
                            backLeftDrive.setPower(backLeftPower);
                            backRightDrive.setPower(backRightPower);
                    }
                    break;
            }

            telemetry.addData("loop time", m_runtime.milliseconds() - lastCycleEndTime);
            telemetry.addData("drive mode", m_driveMode);
            telemetry.addData("goal position", positionController.getGoalPosition());
            telemetry.addData("current path length", positionController.calculateTotalPathLength());
            positionController.addTelemetry();

            //telemetry.addData("launcher velocity", "%4.2f", launcherSpeed);
            //telemetry.addData("status", "Run Time: " + runtime.toString());
            //telemetry.addData("windmill power at launch", windmillPowerAtLaunch);
            //telemetry.addData("launcher velocity at launch", launcherVelocityAtLaunch);
            //telemetry.addData("intake velocity", "%4.2f", intake.getVelocity());
            //telemetry.addData("Axial, Lateral, Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
            //telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);

            telemetry.update();

            lastCycleEndTime = m_runtime.milliseconds();

        }
    }

    /** Moves current toward target: increases are limited to maxStep, decreases and stops are immediate. */
    private double ramp(double current, double target, double maxStep) {
        if (target == 0.0) {
            return 0.0;
        }
        if (current != 0.0 && Math.signum(target) != Math.signum(current)) {
            return 0.0;   // reversing: drop to 0 first
        }
        if (Math.abs(target) <= Math.abs(current)) {
            return target;   // easing off: follow the stick at once
        }
        return current + Math.copySign(Math.min(Math.abs(target - current), maxStep), target);
    }

    protected void modulateIntake() {
        boolean currentShareButtonState = gamepad1.share;
        if (currentShareButtonState && !lastShareButtonState) {
            intake.setPower(0.0);
            if (intake.getDirection() == CRServo.Direction.FORWARD) {
                intake.setDirection(CRServo.Direction.REVERSE);
            } else {
                intake.setDirection(CRServo.Direction.FORWARD);
            }
            intake.setPower(speeds.FULL_SPEED);
            telemetry.addLine("intake reversed");
        }
        lastShareButtonState = currentShareButtonState;
    }

}


