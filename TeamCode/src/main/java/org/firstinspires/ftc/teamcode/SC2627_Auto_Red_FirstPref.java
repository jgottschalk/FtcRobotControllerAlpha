package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Red alliance autonomous: the blue routine mirrored across x = 0.
 * Change the path in SC2627_Auto_Blue_FirstPref; this class picks it up automatically.
 */
@Autonomous(name="SC2627 Auto Red First Preference", group="Robot")
public class SC2627_Auto_Red_FirstPref extends SC2627_Auto_Blue_FirstPref {

    @Override
    protected boolean isRed() { return true; }
}
