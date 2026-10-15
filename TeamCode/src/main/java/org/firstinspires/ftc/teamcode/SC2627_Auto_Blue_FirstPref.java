package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="SC2627 Auto Blue First Preference", group="Robot")
public class SC2627_Auto_Blue_FirstPref extends SC2627Base {

    //move out from wall
    static class FIRST_POSITION {
        public final static double x = 9;
        public final static double y = 64;
        public final static double heading = Math.PI / 4;
    }

    //angle position
    static class SECOND_POSITION {
        public final static double x = 59;
        public final static double y = 27; //72 - 24 -12 - 9
        public final static double heading = Math.PI / 2;
    }

    //rammed position: robot center where its front touches the north wall (72 - 9)
    static class THIRD_POSITION {
        public final static double x = 59;
        public final static double y = 72 - 9;
        public final static double heading = Math.PI / 2;
    }

    //back-up position: back from the wall, so the second ram has a run-up
    static class FOURTH_POSITION {
        public final static double x = 59;
        public final static double y = 27.0;
        public final static double heading = Math.PI / 2;
    }

    static class FIFTH_POSITION {
        public final static double x = 56;
        public final static double y = 72 - 48 - 9;
        public final static double heading = Math.PI / 2;
    }

    //park
    static class END_POSITION {
        public final static double x = 53;
        public final static double y = -30;
        public final static double heading = Math.PI;
    }

    static final double LAUNCH_TIMEOUT_SECONDS = 8.0;
    static final double DRIVE_TIMEOUT_SECONDS = 8.0;

    private final ElapsedTime stepTimer = new ElapsedTime();

    // The routine is written in blue coordinates. The red auto overrides isRed() to return true,
    // and every target is mirrored across x = 0: x -> -x, y unchanged, heading -> PI - heading.
    protected boolean isRed() { return false; }

    protected double mx(double blueX) { return isRed() ? -blueX : blueX; }

    protected double mh(double blueHeading) { return isRed() ? Math.PI - blueHeading : blueHeading; }

    @Override
    public void runOpMode() {

        super.initRobot(isRed() ? GoToPositionController.START_POS.RED : GoToPositionController.START_POS.BLUE, true);
        waitForStart();

        //0. launch artifacts for points
        shoot();

        //1. go south away from wall to make travel easier
        /*
        super.positionController.setTargetPositionAndPath(
                mx(FIRST_POSITION.x),
                FIRST_POSITION.y,
                mh(FIRST_POSITION.heading));
        go();
        */

        //2. go position with intake point north; have sufficient distance for ramming speed
        super.positionController.setTargetPositionAndPath(
                mx(SECOND_POSITION.x),
                SECOND_POSITION.y,
                mh(SECOND_POSITION.heading));
        go();

        //3. ram into north wall
        activateIntakeMode();
        super.positionController.setDriveThroughTarget(
                mx(THIRD_POSITION.x),
                THIRD_POSITION.y,
                mh(THIRD_POSITION.heading),
                0.7);
        go();

        //4. back-up a bit
        super.positionController.setTargetPositionAndPath(
                mx(FOURTH_POSITION.x),
                FOURTH_POSITION.y,
                mh(FOURTH_POSITION.heading));
        go();

        //5. intake again
        super.positionController.setDriveThroughTarget(
                mx(THIRD_POSITION.x),
                THIRD_POSITION.y,
                mh(THIRD_POSITION.heading),
                0.7);
        go();
        deactivateIntakeMode();

        //6. avoid the flower
        super.positionController.setTargetPositionAndPath(
                mx(FIFTH_POSITION.x),
                FIFTH_POSITION.y,
                mh(FIFTH_POSITION.heading));
        go();

        //7. go to end position, park
        super.positionController.setTargetPositionAndPath(
                mx(END_POSITION.x),
                END_POSITION.y,
                mh(END_POSITION.heading));
        go();

        // Keep the saved pose fresh until autonomous ends, so TeleOp gets it even if the routine finished early
        while (opModeIsActive()) {
            updatePose();
            fwOperate.update();
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

}
