package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TEST SC2627 TeleOp Blue First Pref", group="Linear OpMode")
public class SC2627_TeleOp_Blue_FirstPref_Test extends SC2627Base {

    protected final ElapsedTime m_windmillManualTimer = new ElapsedTime();

    boolean lastOptionsButtonState = false;

    protected boolean m_testMode = true;
    protected boolean m_windMillManualMode =false;


    @Override
    public void runOpMode() {

        super.initRobot(GoToPositionController.START_POS.BLUE, true);
        waitForStart();
        activateIntakeMode();

        while (opModeIsActive()) {

            boolean currentOptionsButtonState = gamepad1.options;
            if (currentOptionsButtonState && !lastOptionsButtonState) {
                m_testMode = !m_testMode;
            }
            lastOptionsButtonState = currentOptionsButtonState;

            if (m_testMode) {
                testingOperations();
            } else {
                super.runOpMode();
            }

        }

    }

    protected void testingOperations() {

        if (m_windMillManualMode) {
            telemetry.addData("manual color state",
                    "red: " + color.red() +
                            ", green: " + color.green() +
                            ", blue: " + color.blue() +
                            ", alpha: " + color.alpha() +
                            ", distance: " + color.getDistance(DistanceUnit.CM));
        }

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
