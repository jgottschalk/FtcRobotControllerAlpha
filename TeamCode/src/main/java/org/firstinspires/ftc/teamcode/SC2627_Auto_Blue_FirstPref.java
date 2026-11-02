package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="SC2627 Auto Blue First Preference", group="Robot")
public class SC2627_Auto_Blue_FirstPref extends SC2627Base {

    //angle position
    static class SECOND_POSITION {
        public final static double x = 58;
        public final static double y = 27; //72 - 24 -12 - 9
        public final static double heading = Math.PI / 2;
    }

    //rammed position: robot center where its front touches the north wall (72 - 9)
    static class THIRD_POSITION {
        public final static double x = 58;
        public final static double y = 72 - 9;
        public final static double heading = Math.PI / 2;
    }

    //back-up position: back from the wall, so the second ram has a run-up
    static class FOURTH_POSITION {
        public final static double x = 58;
        public final static double y = 27.0;
        public final static double heading = Math.PI / 2;
    }

    static class FIFTH_POSITION {
        public final static double x = 50;
        public final static double y = 72 - 48 - 9;
        public final static double heading = Math.PI / 2;
    }

    //park
    static class END_POSITION {
        public final static double x = 54;
        public final static double y = -26;
        public final static double heading = Math.PI;
    }

    static final double LAUNCH_TIMEOUT_SECONDS = 8.0;
    static final double DRIVE_TIMEOUT_SECONDS = 8.0;

    private final ElapsedTime stepTimer = new ElapsedTime();

    // The routine is written in blue coordinates. The red auto overrides isRed() to return true,
    // and every target moves to the opposite side of the field: north <-> south, east <-> west,
    // and the robot faces the opposite way (x -> -x, y -> -y, heading -> heading + PI).
    protected boolean isRed() { return false; }

    protected double mx(double blueX) { return isRed() ? -blueX : blueX; }

    protected double my(double blueY) { return isRed() ? -blueY : blueY; }

    protected double mh(double blueHeading) { return isRed() ? blueHeading + Math.PI : blueHeading; }

    @Override
    public void runOpMode() {

        super.initRobot(isRed() ? GoToPositionController.START_POS.RED : GoToPositionController.START_POS.BLUE, true);
        waitForStart();
        startSegmentTiming();

        //1. start position, launch artifacts for points
        shoot();
        endSegment("1. launch");

        //2. go position with intake point north; have sufficient distance for ramming speed
        super.positionController.setTargetAndPlanPath(
                mx(SECOND_POSITION.x),
                my(SECOND_POSITION.y),
                mh(SECOND_POSITION.heading));
        go();
        endSegment("2. to run-up");

        //3. ram into north wall
        activateIntakeMode();
        super.positionController.setDriveThroughTarget(
                mx(THIRD_POSITION.x),
                my(THIRD_POSITION.y),
                mh(THIRD_POSITION.heading),
                0.7);
        go();
        intakeInPlace(2.0);
        endSegment("3. ram + intake");

        //4. back-up a bit
        super.positionController.setTargetAndPlanPath(
                mx(FOURTH_POSITION.x),
                my(FOURTH_POSITION.y),
                mh(FOURTH_POSITION.heading));
        go();
        endSegment("4. back up");

        //5. intake a second, final time
        super.positionController.setDriveThroughTarget(
                mx(THIRD_POSITION.x),
                my(THIRD_POSITION.y),
                mh(THIRD_POSITION.heading),
                0.7);
        go();
        intakeInPlace(1.0);
        deactivateIntakeMode();
        endSegment("5. ram + intake");

        //6. avoid the flower
        super.positionController.setTargetAndPlanPath(
                mx(FIFTH_POSITION.x),
                my(FIFTH_POSITION.y),
                mh(FIFTH_POSITION.heading));
        go();
        endSegment("6. avoid the flower");

        //7. go to end position, park
        super.positionController.setTargetAndPlanPath(
                mx(END_POSITION.x),
                my(END_POSITION.y),
                mh(END_POSITION.heading));
        go();
        endSegment("7. park");

        // Keep the saved pose fresh until autonomous ends, so TeleOp gets it even if the routine finished early.
        // The segment times stay on the Driver Station screen.
        while (opModeIsActive()) {
            updatePose();
            fwOperate.update();
            addSegmentTelemetry();
            telemetry.update();
        }

    }

    // Read odometry and save the pose for TeleOp. Called every loop so the latest pose is saved
    // even if autonomous is stopped by the timer partway through a step.
    private void updatePose() {
        positionController.odometry();
        // Only save a good reading. A Pinpoint read that fails (for example while the OpMode is being
        // stopped) comes back as zeros, and saving that would hand TeleOp a pose of (0, 0).
        if (!isStopRequested() && positionController.isPinpointReady()) {
            PoseStorage.save(positionController.getX(), positionController.getY(), positionController.getHeading());
        }
    }

    protected void shoot() {
        // Same order as TeleOp: intake off, flywheel on, then launch.
        // setSpinning(true) rather than activateOuttakeMode(): that toggles, so the second
        // shoot() would turn the flywheel off and wait forever for it to be ready.
        deactivateIntakeMode();
        fwOperate.setSpinning(true);
        fwOperate.launch();
        stepTimer.reset();
        while (opModeIsActive() && !fwOperate.idle()) {
            if (stepTimer.seconds() > LAUNCH_TIMEOUT_SECONDS) {
                fwOperate.stop();
                fwOperate.stopWindmill();
                break;
            }
            updatePose();
            fwOperate.update();
            fwOperate.addTelemetry(telemetry);
            telemetry.update();
        }
        fwOperate.stop();   // launch done: spin the flywheel down
    }

    protected void go() {
        stepTimer.reset();
        while (opModeIsActive() && stepTimer.seconds() < DRIVE_TIMEOUT_SECONDS) {
            updatePose();
            boolean arrived = positionController.autoAssistDrive();
            positionController.logState("go");
            if (arrived) {
                break;
            }
            fwOperate.update();
            positionController.addTelemetry();
            telemetry.update();
        }
        positionController.stopMotors();
    }

    protected void intakeInPlace(double seconds) {

        positionController.stopMotors();

        stepTimer.reset();
        while (opModeIsActive() && stepTimer.seconds() < seconds) {
            updatePose();
            fwOperate.update();
            telemetry.update();
        }

    }

}
