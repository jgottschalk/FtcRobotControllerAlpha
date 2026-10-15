package org.firstinspires.ftc.teamcode;

/**
 * Carries the robot's field pose from autonomous into TeleOp. Static fields survive from one
 * OpMode to the next while the Robot Controller app keeps running.
 *
 * Autonomous calls save() every loop. TeleOp calls hasPose() at init and, if true, starts from the
 * last pose autonomous saved instead of the start position.
 */
public class PoseStorage {

    private static boolean saved = false;
    private static double x = 0.0;        // inches
    private static double y = 0.0;        // inches
    private static double heading = 0.0;  // radians

    public static void save(double xInches, double yInches, double headingRadians) {
        x = xInches;
        y = yInches;
        heading = headingRadians;
        saved = true;
    }

    public static boolean hasPose() {
        return saved;
    }

    public static double getX() { return x; }
    public static double getY() { return y; }
    public static double getHeading() { return heading; }
}
