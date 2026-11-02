package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Basic Pedro Pathing test auto (blue). Uses the same field coordinates as the existing autos:
 * inches from the field centre, heading in degrees, 90 = facing north.
 *
 * Deliberately does NOT extend SC2627Base: initRobot() creates a GoToPositionController, which also
 * configures the Pinpoint and would conflict with Pedro.
 */
@Autonomous(name = "SC2627 Pedro Test Auto (Blue)", group = "Robot")
public class SC2627_Auto_Pedro_Test extends LinearOpMode {

    private final PoseFactory p = PoseFactory.degrees();

    // Same start as SC2627_Auto_Blue_FirstPref
    private final Pose start = p.of(9, 65, 90);
    // 1. straight south, away from the wall
    private final Pose awayFromWall = p.of(9, 45, 90);
    // 2. curve east and south, clear of the east hazard (passes about (35, 37) and (44, 28))
    private final Pose curveControl = p.of(40, 45, 90);
    private final Pose end = p.of(50, 15, 90);

    private Follower follower;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(start);

        Path leg1 = line(start, awayFromWall).constant(start);              // straight, keep facing north
        Path leg2 = curve(awayFromWall, curveControl, end).constant(start); // Bezier curve, keep facing north

        telemetry.addLine("Pedro test auto ready");
        telemetry.update();
        waitForStart();

        followAndWait(leg1, "1. straight");
        followAndWait(leg2, "2. curve");

        // Pedro holds the end position by default; keep updating and saving the pose until auto ends
        while (opModeIsActive()) {
            follower.update();
            savePoseAndShow("done");
        }
    }

    /** Starts a path and keeps the follower updated until it has finished. */
    private void followAndWait(Path path, String name) {
        follower.follow(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            savePoseAndShow(name);
        }
    }

    /** Saves the pose for TeleOp (same PoseStorage as the other autos) and shows it on the Driver Station. */
    private void savePoseAndShow(String step) {
        Pose pose = follower.pose();
        PoseStorage.save(pose.x(), pose.y(), pose.heading());   // heading() is in radians, as PoseStorage expects
        telemetry.addData("Step", step);
        telemetry.addData("Pose (X, Y, Heading)", "(%.1f, %.1f, %.1f deg)",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }
}
