
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.flywheel.FlywheelOperate;

/*
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * gamepad X: SQUARE BUTTON
 * gamepad A: X BUTTON
 * gamepad Y: TRIANGLE BUTTON
 * gamepad B: CIRCLE BUTTON
*/

@TeleOp(name="SC2627 TeleOp: Base for Testing", group="Linear OpMode")
//@Disabled
public class SC2627Base extends LinearOpMode {

    private enum xTakeMode {
        INTAKE,
        POSITIONING,
        OUTTAKE
    }

    private enum DriveMode {
        MANUAL,
        AUTO_ASSIST
    }

    //timers
    private final ElapsedTime m_windmillManualTimer = new ElapsedTime();
    private final ElapsedTime m_runtime = new ElapsedTime();
    double lastCycleEndTime = 0.0;

    //power draws
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    private DcMotorEx intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private CRServo windMill = null;
    private RevColorSensorV3 color = null;
    private VoltageSensor voltageSensor = null;
    private GoBildaPinpointDriver pinpoint = null;

    //controllers
    protected FlywheelOperate fwOperate = null;
    protected GoToPositionController positionController = null;

    //modes
    private DriveMode m_driveMode = DriveMode.MANUAL;
    private xTakeMode m_xTakeMode = xTakeMode.INTAKE;
    private boolean m_testMode = false;
    private boolean m_windMillManualMode =false;

    //buttons
    boolean lastYButtonState = false;
    boolean lastAButtonState = false;
    boolean lastShareButtonState = false;
    boolean lastUpButtonState = false;
    boolean lastDownButtonState = false;

    //positions (default to blue)
    GoToPositionController.START_POS startPos = null;

    //File soundFile;

    public void initRobot(GoToPositionController.START_POS ps, boolean firstPreference) {

        startPos = ps;

        m_xTakeMode = xTakeMode.INTAKE;

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

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake on zero power. Must match GoToPositionTuningOpMode so tuned gains behave the same here.
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launcher.setZeroPowerBehavior(BRAKE);
        //launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setPower(speeds.STOP_SPEED);
        rightFeeder.setPower(speeds.STOP_SPEED);

        windMill.setDirection(CRServo.Direction.FORWARD);
        windMill.setPower(speeds.STOP_SPEED);

        leftFeeder.setDirection(CRServo.Direction.FORWARD);
        rightFeeder.setDirection(CRServo.Direction.REVERSE);

        activateIntakeMode();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        positionController = new GoToPositionController(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, pinpoint, telemetry, startPos, firstPreference);

        // TeleOp continues from where autonomous left the robot, if autonomous saved a pose recently.
        // Autonomous always starts from the start position.
        boolean isAutonomous = getClass().isAnnotationPresent(Autonomous.class);
        if (!isAutonomous && PoseStorage.isFresh()) {
            positionController.setPose(PoseStorage.getX(), PoseStorage.getY(), PoseStorage.getHeading());
            PoseStorage.clear();
            telemetry.addData("Start pose", "from autonomous (%.1f, %.1f, %.1f deg)",
                    PoseStorage.getX(), PoseStorage.getY(), Math.toDegrees(PoseStorage.getHeading()));
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

        //default to blue for testin purposes
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

            //if (gamepad1.share) {
            //m_testMode = !m_testMode;
            //}

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

            boolean currentUpButtonState = gamepad1.dpad_up;
            if (currentUpButtonState && !lastUpButtonState) {
                positionController.setGoalPosition(GoToPositionController.GOAL_POS.NEAR);
            }
            lastUpButtonState = currentUpButtonState;

            boolean currentDownButtonState = gamepad1.dpad_down;
            if (currentDownButtonState && !lastDownButtonState) {
                positionController.setGoalPosition(GoToPositionController.GOAL_POS.FAR);
            }
            lastDownButtonState = currentDownButtonState;

            if (m_windMillManualMode) {
                telemetry.addData("manual color state",
                        "red: " + color.red() +
                                ", green: " + color.green() +
                                ", blue: " + color.blue() +
                                ", alpha: " + color.alpha() +
                                ", distance: " + color.getDistance(DistanceUnit.CM));
            }

            // windmill test mode
            if (gamepad1.bWasPressed()) { //circle button
                if (!m_windMillManualMode) {
                    windMill.setPower(0.1);
                    launcher.setPower(0);
                    intake.setPower(0);
                    m_windMillManualMode = true;
                    m_windmillManualTimer.reset();
                } else {
                    windMill.setPower(0.0);
                    m_windMillManualMode = false;
                }
            }

            if (m_windMillManualMode && (m_windmillManualTimer.seconds() > 10.0)) {
                windMill.setPower(speeds.STOP_SPEED);
                m_windMillManualMode = false;
            }


            if (((gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased())
                    && (m_xTakeMode == xTakeMode.OUTTAKE))
            ) {

                fwOperate.launch();

            } else if ((gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased())
                    && (m_xTakeMode == xTakeMode.INTAKE)) {
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
                        // Don't block here: the flywheel and the rest of the loop must keep running.
                        // Current vs. target pose stays visible via addTelemetry() every loop.
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
                    if (currentAButtonState && !lastAButtonState) {
                        positionController.goScore();
                        m_driveMode = DriveMode.AUTO_ASSIST;
                    }
                    lastAButtonState = currentAButtonState;

                    if (m_driveMode == DriveMode.MANUAL) {
                        if (m_testMode) {
                            operateManualTests();
                        } else {
                            frontLeftDrive.setPower(frontLeftPower * speeds.DEFAULT_SPEED);
                            frontRightDrive.setPower(frontRightPower * speeds.DEFAULT_SPEED);
                            backLeftDrive.setPower(backLeftPower * speeds.DEFAULT_SPEED);
                            backRightDrive.setPower(backRightPower * speeds.DEFAULT_SPEED);
                        }
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
            //telemetry.addData("drive power", powerCoefficient);
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

    private void operateManualTests() {

        frontLeftDrive.setPower(gamepad1.x ? 1.0 : 0.0);  // SQUARE
        backLeftDrive.setPower(gamepad1.a ? 1.0 : 0.0);  // X
        frontRightDrive.setPower(gamepad1.y ? 1.0 : 0.0);  // TRIANGLE
        backRightDrive.setPower(gamepad1.b ? 1.0 : 0.0);  // CIRCLE

        if (gamepad1.dpad_up) {
            telemetry.addLine("feeder test");
            if (leftFeeder.getPower() > 0) {
                leftFeeder.setPower(-1.0);
                rightFeeder.setPower(-1.0);
            } else {
                leftFeeder.setPower(1.0);
                rightFeeder.setPower(1.0);
            }
        }
    }

}