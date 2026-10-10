package org.firstinspires.ftc.teamcode;

/**
 * Carries the robot's field pose from autonomous into TeleOp. Static fields survive from one
 * OpMode to the next while the Robot Controller app keeps running.
 *
 * Autonomous calls save() every loop. TeleOp calls isFresh() at init and, if true, starts from the
 * saved pose instead of the start position, then calls clear() so the pose is only used once.
 */
public class PoseStorage {

    // TeleOp only uses a pose saved this recently. Anything older is from an earlier run.
    public static final double MAX_AGE_SECONDS = 30.0;

    private static boolean saved = false;
    private static double x = 0.0;        // inches
    private static double y = 0.0;        // inches
    private static double heading = 0.0;  // radians
    private static long savedAtNanos = 0;

    public static void save(double xInches, double yInches, double headingRadians) {
        x = xInches;
        y = yInches;
        heading = headingRadians;
        savedAtNanos = System.nanoTime();
        saved = true;
    }

    public static boolean isFresh() {
        return saved && (System.nanoTime() - savedAtNanos) / 1e9 <= MAX_AGE_SECONDS;
    }

    public static void clear() {
        saved = false;
    }

    public static double getX() { return x; }
    public static double getY() { return y; }
    public static double getHeading() { return heading; }
}
