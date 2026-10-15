package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SC2627 TeleOp Blue Second Pref", group="Linear OpMode")
public class SC2627_TeleOp_Blue_SecondPref extends SC2627Base {

    @Override
    public void runOpMode() {
        super.initRobot(GoToPositionController.START_POS.BLUE, false);
        waitForStart();
        activateIntakeMode();
        super.runOpMode();
    }

}
