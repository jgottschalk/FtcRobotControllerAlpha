
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

import org.firstinspires.ftc.teamcode.flywheel.FlywheelOperate;

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

            double max;

            double axial   = (Math.abs(gamepad1.left_stick_y)  > 0.08) ? -gamepad1.left_stick_y : 0.0;
            double lateral = (Math.abs(gamepad1.left_stick_x)  > 0.08) ?  gamepad1.left_stick_x : 0.0;
            double yaw     = (Math.abs(gamepad1.right_stick_x) > 0.08) ?  gamepad1.right_stick_x : 0.0;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
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

                            frontLeftDrive.setPower(frontLeftPower * speeds.DEFAULT_SPEED);
                            frontRightDrive.setPower(frontRightPower * speeds.DEFAULT_SPEED);
                            backLeftDrive.setPower(backLeftPower * speeds.DEFAULT_SPEED);
                            backRightDrive.setPower(backRightPower * speeds.DEFAULT_SPEED);
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


