package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Red alliance autonomous: the blue routine mirrored across x = 0.
 * Change the path in SC2627_Auto_Blue_FirstPref; this class picks it up automatically.
 */
@Autonomous(name="SC2627 Auto Red Second Preference", group="Robot")
public class SC2627_Auto_Red_SecondPref extends SC2627_Auto_Red_FirstPref {

    @Override
    protected boolean isRed() { return true; }

    static class FA_POSITION {
        public final static double x = -74 + 9 + 12;
        public final static double y = 0;
        public final static double heading = Math.PI / 2;
    }

    //park
    static class ENDO_POSITION {
        public final static double x = -74 + 12;
        public final static double y = 40;
        public final static double heading = 0.0;
    }

    @Override
    public void runOpMode() {

        super.initRobot(isRed() ? GoToPositionController.START_POS.RED : GoToPositionController.START_POS.BLUE, false);
        waitForStart();

        // avoid the flower
        // FA_POSITION and ENDO_POSITION are already red-side field coordinates, so they are used as-is.
        // (mx/my/mh convert blue coordinates to red; applying them here would send the robot to the blue side.)
        positionController.setTargetPositionAndPath(
                FA_POSITION.x,
                FA_POSITION.y,
                FA_POSITION.heading);
        go();

        // go to end position, park
        positionController.setTargetPositionAndPath(
                ENDO_POSITION.x,
                ENDO_POSITION.y,
                ENDO_POSITION.heading);
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


}
