

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
 *
*/

@TeleOp(name="Short Circuits: 2627 Test", group="Linear OpMode")
//@Disabled
public class BasicOmniOpMode_Linear_2627Test extends OpMode {

    //timing telemetry
    private ElapsedTime windmillTimer = new ElapsedTime();

    private ElapsedTime windmillManualTimer = new ElapsedTime();

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    //drive train: 4 motors
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    //outtake: 1 launcher motor
    private DcMotorEx launcher = null;

    //intake: 1 motor and 2 servos
    private DcMotorEx intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    //routing/channeling: 1 servo ("windmill")
    private CRServo windMill = null;

    private double launcherSpeed = 0.0;

    private RevColorSensorV3 color = null;

    private String colorState = "";

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        REALIGNING
    }

    private enum RobotMode {
        INTAKE,
        OUTTAKE
    }
    private LaunchState launchState;

    private RobotMode robotMode;

    private boolean testMode = false;

    //private boolean pauseIntake = false;

    private boolean windMillManualMode =false;

    double powerCoefficient = speeds.DEFAULT_SPEED;
    boolean lastYButtonState = false;
    boolean toggleYButtonState = false;
    double launcherVelocityAtLaunch = 0.0;

    double windmillPowerAtLaunch = 0.0;

    double lastCycleEndTime = 0.0;

    @Override
    public void init() {

        launchState = LaunchState.IDLE;
        robotMode = RobotMode.INTAKE;

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

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setPower(speeds.STOP_SPEED);
        rightFeeder.setPower(speeds.STOP_SPEED);

        windMill.setDirection(CRServo.Direction.FORWARD);
        windMill.setPower(speeds.STOP_SPEED);

        leftFeeder.setDirection(CRServo.Direction.FORWARD);
        rightFeeder.setDirection(CRServo.Direction.REVERSE);

        activateIntakeMode();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();

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
        launcher.setVelocity(speeds.LAUNCHER_TARGET_VELOCITY);
    }

    protected void deactivateOuttakeMode () {
        launcher.setVelocity(speeds.STOP_SPEED);
    }


    @Override
    public void loop() {

        boolean currentYButtonState = gamepad1.y;

        if (currentYButtonState && !lastYButtonState) {
            toggleYButtonState = true;
        } else {
            toggleYButtonState = false;
        }

        lastYButtonState = currentYButtonState;

        if (toggleYButtonState) {
            if (robotMode == RobotMode.INTAKE) { // activate outtake
                robotMode = RobotMode.OUTTAKE;
                deactivateIntakeMode();
                activateOuttakeMode();
            } else { //activate intake
                robotMode = RobotMode.INTAKE;
                deactivateOuttakeMode();
                activateIntakeMode();
            }
        }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

        if (gamepad1.share) {
            testMode = !testMode;
        }

        if (windMillManualMode) {
            colorState =
                    "red: " + color.red() +
                            ", green: " + color.green() +
                            ", blue: " + color.blue() +
                            ", alpha: " + color.alpha() +
                            ", distance: " + color.getDistance(DistanceUnit.CM);
        }

        // windmill test mode
        if (gamepad1.b) { //circle button
            if (!windMillManualMode) {
                windMill.setPower(0.1);
                launcher.setPower(0);
                intake.setPower(0);
                windMillManualMode = true;
                windmillManualTimer.reset();
            } else {
                windMill.setPower(0.0);
                windMillManualMode = false;
            }
        }

        if (windMillManualMode && (windmillManualTimer.seconds() > 10.0)) {
                windMill.setPower(speeds.STOP_SPEED);
                windMillManualMode = false;
        }

        if (testMode) {
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // SQUARE
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // X
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // TRIANGLE
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // CIRCLE

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

        } else {
            frontLeftPower = frontLeftPower * powerCoefficient;
            frontRightPower = frontRightPower * powerCoefficient;
            backLeftPower = backLeftPower * powerCoefficient;
            backRightPower = backRightPower * powerCoefficient;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);

        if(     ((gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased() )
                && (robotMode == RobotMode.OUTTAKE))
                || launchState != LaunchState.IDLE
        ) {
            launch();
        } else if ((gamepad1.rightBumperWasReleased() || gamepad1.leftBumperWasReleased() )
                   && (robotMode == RobotMode.INTAKE)) {
                gamepad1.rumbleBlips(3);
        }

        //telemetry.addData("status", "Run Time: " + runtime.toString());
        telemetry.addData("loop time", runtime.milliseconds() - lastCycleEndTime);
        //telemetry.addData("drive power", powerCoefficient);
        telemetry.addData("launch state", launchState);
        telemetry.addData("launcher velocity", "%4.2f", launcherSpeed);
        telemetry.addData("launcher velocity at launch", launcherVelocityAtLaunch);
        //telemetry.addData("windmill power at launch", windmillPowerAtLaunch);
        //telemetry.addData("intake velocity", "%4.2f", intake.getVelocity());
        telemetry.addData("color state", colorState);

                //telemetry.addData("Axial, Lateral, Yaw", "%4.2f, %4.2f, %4.2f", axial, lateral, yaw);
        //telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);

        telemetry.update();

        lastCycleEndTime = runtime.milliseconds();
    }

    void launch() {

        launcherSpeed = launcher.getVelocity();

        switch (launchState) {

            case IDLE:

                launchState = LaunchState.SPIN_UP;

                break;

            case SPIN_UP:

                if (launcher.getVelocity() < speeds.LAUNCHER_TARGET_VELOCITY) {
                    //no op
                } else {
                    launchState = LaunchState.LAUNCH;
                    windMill.setPower(0.5);
                }

                break;

            case LAUNCH:

                launchState = LaunchState.LAUNCHING;
                windmillTimer.reset();
                launcherVelocityAtLaunch = launcher.getVelocity();
                windmillPowerAtLaunch = windMill.getPower();
                break;

            case LAUNCHING:

                intake.setPower(speeds.FULL_SPEED);

                if (windmillTimer.seconds() > speeds.WINDMILL_TIME_SECONDS) {
                    launchState = LaunchState.REALIGNING;
                    windMill.setPower(0.1);
                    intake.setPower(0.0);
                }
                break;

            case REALIGNING:

                final double buffer = 0.5;

                boolean stopWindmill = (windmillTimer.seconds() > buffer + speeds.WINDMILL_TIME_SECONDS) &&
                                       //(color.blue() > 43) &&
                                       //(color.blue() < 51) &&
                                       (color.alpha() < 45) &&
                                       (color.getDistance(DistanceUnit.CM) > 5.8) &&
                                       (color.getDistance(DistanceUnit.CM) < 6.1);

                    colorState =
                                    "red: " + color.red() +
                                    ", green: " + color.green() +
                                    ", blue: " + color.blue() +
                                    ", alpha: " + color.alpha() +
                                    ", distance: " + color.getDistance(DistanceUnit.CM);

                    if (stopWindmill) {
                        windMill.setPower(speeds.STOP_SPEED);
                        launchState = LaunchState.IDLE;
                    }

                break;
        }

    }

}