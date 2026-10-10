package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

public class FieldObstaclePlanner {

    // Dead zone
    public static final double X_MIN = -35.0;
    public static final double X_MAX = 35.0;
    public static final double Y_MIN = -31.5;
    public static final double Y_MAX = 31.5;

    // Generous clearance outside the box so robot chassis (radius ~9") never clips
    private static final double CLEARANCE = 10.0;

    private static final double LEFT_X   = X_MIN - CLEARANCE; // -45.0
    private static final double RIGHT_X  = X_MAX + CLEARANCE; // +45.0
    private static final double BOTTOM_Y = Y_MIN - CLEARANCE; // -41.5
    private static final double TOP_Y    = Y_MAX + CLEARANCE; // +41.5

    public static class Waypoint {
        public final double x;
        public final double y;
        public final double heading;

        public Waypoint(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public static List<Waypoint> planPath(double startX, double startY,
                                          double targetX, double targetY,
                                          double targetHeading) {
        List<Waypoint> path = new ArrayList<>();

        if (intersectsDeadZone(startX, startY, targetX, targetY)) {
            double flankX = (startX < 0) ? LEFT_X : RIGHT_X;
            double flankY = (targetY < 0) ? BOTTOM_Y : TOP_Y;

            double corner1X = flankX;
            // Already beside the box (outside its X span): just step sideways into the flank lane.
            // Otherwise (above or below the box): first move out to the clearance line.
            double corner1Y;
            if (Math.abs(startX) >= X_MAX) {
                corner1Y = startY;
            } else {
                corner1Y = (startY < 0) ? Math.min(startY, BOTTOM_Y) : Math.max(startY, TOP_Y);
            }

            double corner2X = flankX;
            double corner2Y = flankY;

            if (Math.abs(startX) > Math.abs(flankX) - 2.0) {
                path.add(new Waypoint(corner2X, corner2Y, targetHeading));
            } else {
                path.add(new Waypoint(corner1X, corner1Y, targetHeading));
                path.add(new Waypoint(corner2X, corner2Y, targetHeading));
            }
        }

        path.add(new Waypoint(targetX, targetY, targetHeading));
        return path;
    }

    public static boolean intersectsDeadZone(double x1, double y1, double x2, double y2) {
        if (isInside(x1, y1) || isInside(x2, y2)) {
            return true;
        }

        double dx = x2 - x1;
        double dy = y2 - y1;

        double p1 = -dx, q1 = x1 - X_MIN;
        double p2 =  dx, q2 = X_MAX - x1;
        double p3 = -dy, q3 = y1 - Y_MIN;
        double p4 =  dy, q4 = Y_MAX - y1;

        double[] p = {p1, p2, p3, p4};
        double[] q = {q1, q2, q3, q4};

        double u1 = 0.0;
        double u2 = 1.0;

        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) return false;
            } else {
                double t = q[i] / p[i];
                if (p[i] < 0) {
                    if (t > u2) return false;
                    if (t > u1) u1 = t;
                } else {
                    if (t < u1) return false;
                    if (t < u2) u2 = t;
                }
            }
        }
        return u1 <= u2;
    }

    private static boolean isInside(double x, double y) {
        return (x > X_MIN && x < X_MAX && y > Y_MIN && y < Y_MAX);
    }
}