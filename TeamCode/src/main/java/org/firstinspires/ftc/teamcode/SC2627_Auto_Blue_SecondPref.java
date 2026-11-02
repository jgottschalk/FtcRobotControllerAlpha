package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="SC2627 Auto Blue Second Preference", group="Robot")
public class SC2627_Auto_Blue_SecondPref extends SC2627Base {

    static class FIFTH_POSITION {
        public final static double x = 45;
        public final static double y = 72 - 48 - 9;
        public final static double heading = Math.PI / 2;
    }

    //park
    static class END_POSITION {
        public final static double x = 54;
        public final static double y = -40;
        public final static double heading = Math.PI;
    }

    static final double DRIVE_TIMEOUT_SECONDS = 8.0;

    private final ElapsedTime stepTimer = new ElapsedTime();

    // The routine is written in blue coordinates. The red auto overrides isRed() to return true,
    // and every target is mirrored across x = 0: x -> -x, y unchanged, heading -> PI - heading.
    protected boolean isRed() { return false; }

    protected double mx(double blueX) { return isRed() ? -blueX : blueX; }

    protected double mh(double blueHeading) { return isRed() ? Math.PI - blueHeading : blueHeading; }

    @Override
    public void runOpMode() {

        super.initRobot(isRed() ? GoToPositionController.START_POS.RED : GoToPositionController.START_POS.BLUE, false);
        waitForStart();
        startSegmentTiming();

        //1. avoid the flower
        super.positionController.setTargetAndPlanPath(
                mx(FIFTH_POSITION.x),
                FIFTH_POSITION.y,
                mh(FIFTH_POSITION.heading));
        go();
        endSegment("1. avoid the flower");

        //2. go to end position, park
        super.positionController.setTargetAndPlanPath(
                mx(END_POSITION.x),
                END_POSITION.y,
                mh(END_POSITION.heading));
        go();
        endSegment("2. park");

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
