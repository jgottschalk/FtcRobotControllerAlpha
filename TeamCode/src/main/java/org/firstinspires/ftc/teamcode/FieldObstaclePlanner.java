package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/**
 * Plans a route around the two hazard rectangles at the base of the hive. Each is a narrow
 * north-south rectangle (2" wide, 39" tall) centred on y = 0, with a 45" gap between them that the
 * robot may drive through.
 *
 * Each rectangle is grown by a safety margin (the robot's centre must stay outside the grown
 * rectangle), and the shortest route is found through the corners of the grown rectangles.
 */
public class FieldObstaclePlanner {

    // Hazard rectangles (inches, field coordinates). The gap between their inner edges is 45".
    public static final double HAZARD_WIDTH = 2.0;
    public static final double HAZARD_HEIGHT = 39.0;
    public static final double HAZARD_GAP = 45.0;

    private static final double INNER_X = HAZARD_GAP / 2.0;          // 22.5
    private static final double OUTER_X = INNER_X + HAZARD_WIDTH;    // 24.5
    private static final double HALF_HEIGHT = HAZARD_HEIGHT / 2.0;   // 19.5

    // Safety margin around each rectangle, between it and the robot's centre (robot half-width ~9")
    public static final double CLEARANCE_SIDES = 10.0;       // east and west sides
    public static final double CLEARANCE_TOP_BOTTOM = 12.0;  // north and south ends

    // Grown rectangles: {xMin, xMax, yMin, yMax}
    private static final double[][] ZONES = {
            { INNER_X - CLEARANCE_SIDES,  OUTER_X + CLEARANCE_SIDES,     // east: 12.5 to 34.5
                    -HALF_HEIGHT - CLEARANCE_TOP_BOTTOM, HALF_HEIGHT + CLEARANCE_TOP_BOTTOM },  // -31.5 to 31.5
            { -OUTER_X - CLEARANCE_SIDES, -INNER_X + CLEARANCE_SIDES,    // west: -34.5 to -12.5
                    -HALF_HEIGHT - CLEARANCE_TOP_BOTTOM, HALF_HEIGHT + CLEARANCE_TOP_BOTTOM },
    };

    // Route corners are placed this far outside the grown rectangles, so a route along an edge is clear
    private static final double CORNER_OFFSET = 0.5;

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

    /**
     * Returns the waypoints from the start to the target (the target is always the last one).
     * A straight line is used when it is clear; otherwise the shortest route via zone corners.
     */
    public static List<Waypoint> planPath(double startX, double startY,
                                          double targetX, double targetY,
                                          double targetHeading) {
        // Nodes: 0 = start, 1 = target, then the corners of every zone
        List<double[]> nodes = new ArrayList<>();
        nodes.add(new double[]{startX, startY});
        nodes.add(new double[]{targetX, targetY});
        for (double[] z : ZONES) {
            double x0 = z[0] - CORNER_OFFSET, x1 = z[1] + CORNER_OFFSET;
            double y0 = z[2] - CORNER_OFFSET, y1 = z[3] + CORNER_OFFSET;
            double[][] corners = {{x0, y0}, {x1, y0}, {x0, y1}, {x1, y1}};
            for (double[] c : corners) {
                if (!insideAnyZone(c[0], c[1])) {
                    nodes.add(c);
                }
            }
        }

        // Shortest route over clear straight segments (Dijkstra; only a handful of nodes)
        int n = nodes.size();
        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] done = new boolean[n];
        for (int i = 0; i < n; i++) {
            dist[i] = Double.MAX_VALUE;
            prev[i] = -1;
        }
        dist[0] = 0.0;

        for (int iter = 0; iter < n; iter++) {
            int u = -1;
            for (int i = 0; i < n; i++) {
                if (!done[i] && dist[i] < Double.MAX_VALUE && (u < 0 || dist[i] < dist[u])) {
                    u = i;
                }
            }
            if (u < 0 || u == 1) {
                break;
            }
            done[u] = true;
            for (int v = 0; v < n; v++) {
                if (done[v] || v == u) {
                    continue;
                }
                if (!segmentClear(nodes.get(u), nodes.get(v), u == 0, v == 1)) {
                    continue;
                }
                double d = dist[u] + Math.hypot(nodes.get(v)[0] - nodes.get(u)[0], nodes.get(v)[1] - nodes.get(u)[1]);
                if (d < dist[v]) {
                    dist[v] = d;
                    prev[v] = u;
                }
            }
        }

        List<Waypoint> path = new ArrayList<>();
        if (prev[1] < 0) {
            // No clear route found (should not happen on this field): drive straight to the target
            path.add(new Waypoint(targetX, targetY, targetHeading));
            return path;
        }
        List<Integer> order = new ArrayList<>();
        for (int v = 1; v != 0; v = prev[v]) {
            order.add(0, v);
        }
        for (int v : order) {
            path.add(new Waypoint(nodes.get(v)[0], nodes.get(v)[1], targetHeading));
        }
        return path;
    }

    /**
     * True if the straight segment a-b doesn't pass through any zone. A zone that contains the
     * start (fromStart) or the target (toTarget) is ignored for that segment, so the robot can
     * always leave a zone it is already in, or reach a target that is set inside one.
     */
    private static boolean segmentClear(double[] a, double[] b, boolean fromStart, boolean toTarget) {
        for (double[] z : ZONES) {
            if (fromStart && inside(z, a[0], a[1])) {
                continue;
            }
            if (toTarget && inside(z, b[0], b[1])) {
                continue;
            }
            if (segmentHitsZone(z, a[0], a[1], b[0], b[1])) {
                return false;
            }
        }
        return true;
    }

    /** Liang-Barsky test: does the segment pass through the inside of the zone? */
    private static boolean segmentHitsZone(double[] z, double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {x1 - z[0], z[1] - x1, y1 - z[2], z[3] - y1};
        double u1 = 0.0;
        double u2 = 1.0;
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] <= 0) return false;
            } else {
                double t = q[i] / p[i];
                if (p[i] < 0) {
                    if (t > u1) u1 = t;
                } else {
                    if (t < u2) u2 = t;
                }
            }
        }
        return u1 < u2;
    }

    private static boolean inside(double[] z, double x, double y) {
        return x > z[0] && x < z[1] && y > z[2] && y < z[3];
    }

    private static boolean insideAnyZone(double x, double y) {
        for (double[] z : ZONES) {
            if (inside(z, x, y)) {
                return true;
            }
        }
        return false;
    }
}
