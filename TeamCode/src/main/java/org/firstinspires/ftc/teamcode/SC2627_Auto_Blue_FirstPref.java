package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="SC2627 Auto Blue First Preference", group="Robot")
public class SC2627_Auto_Blue_FirstPref extends SC2627Base {

    static class FIRST_POSITION {
        public final static double x = 47;
        public final static double y = 58;
        public final static double heading = Math.PI / 3;
    }

    static class SECOND_POSITION {
        public final static double x = 47;
        public final static double y = 61;
        public final static double heading = Math.PI / 3;
    }

    static class THIRD_POSITION {
        public final static double x = 58;
        public final static double y = 61;
        public final static double heading = 0;//Math.PI / 3;
    }

    static class END_POSITION {
        public final static double x = 54;
        public final static double y = -30;
        public final static double heading = (3 * Math.PI) / 2;
    }

    static final double LAUNCH_TIMEOUT_SECONDS = 8.0;
    static final double DRIVE_TIMEOUT_SECONDS = 8.0;

    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        super.initRobot(GoToPositionController.START_POS.BLUE, true);
        waitForStart();

        //0. launch artifacts for points
        shoot();

        super.positionController.setTargetPositionAndPath(
                FIRST_POSITION.x,
                FIRST_POSITION.y,
                FIRST_POSITION.heading);
        go();

        //1. move to intake at first position
        activateIntakeMode();
        super.positionController.setTargetPositionAndPath(
                SECOND_POSITION.x,
                SECOND_POSITION.y,
                SECOND_POSITION.heading);
        go();

        //2. keep going to sweep artifacts
        super.positionController.setTargetPositionAndPath(
                THIRD_POSITION.x,
                THIRD_POSITION.y,
                THIRD_POSITION.heading);
        go();
        deactivateIntakeMode();

        //3. go to end position, park
        super.positionController.setTargetPositionAndPath(
                END_POSITION.x,
                END_POSITION.y,
                END_POSITION.heading);
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
        PoseStorage.save(positionController.getX(), positionController.getY(), positionController.getHeading());
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
