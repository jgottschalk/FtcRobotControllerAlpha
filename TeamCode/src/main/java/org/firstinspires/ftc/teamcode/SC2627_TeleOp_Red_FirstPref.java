package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SC2627 TeleOp Red First Pref", group="Linear OpMode")
public class SC2627_TeleOp_Red_FirstPref extends SC2627Base {

    @Override
    public void runOpMode() {
        super.initRobot(GoToPositionController.START_POS.RED, true);
        waitForStart();
        activateIntakeMode();
        super.runOpMode();
    }

}
