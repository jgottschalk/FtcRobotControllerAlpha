package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

public class GoToPositionController {

    static class POSITION_START_BLUE_FIRST_PREFERENCE {
        public final static double x = 9;
        public final static double y = 65; //74 - 9;
        public final static double heading = (Math.PI)/2; //(3 * Math.PI)/2;
    }

    static class POSITION_START_BLUE_SECOND_PREFERENCE {
        public final static double x = 9 + 24;
        public final static double y = 65;
        public final static double heading = (Math.PI)/2;
    }

    // Red positions are the blue ones on the opposite side of the field: north <-> south,
    // east <-> west, facing the opposite way (x -> -x, y -> -y, heading -> heading + PI).
    // They are defined from the blue values so the two alliances can't drift apart.

    static class POSITION_START_RED_FIRST_PREFERENCE {
        public final static double x = -POSITION_START_BLUE_FIRST_PREFERENCE.x;
        public final static double y = -POSITION_START_BLUE_FIRST_PREFERENCE.y;
        public final static double heading = normalizeAngle(POSITION_START_BLUE_FIRST_PREFERENCE.heading + Math.PI);
    }

    static class POSITION_START_RED_SECOND_PREFERENCE {
        public final static double x = -74 + 24 + 9; //-41
        public final static double y = -74 + 9; //-65
        public final static double heading = (3 * Math.PI) /2;
    }

    static class POSITION_END_BLUE {
        public final static double x = 63;
        public final static double y = -39;
        public final static double heading = 0.0;
    }

    static class POSITION_END_RED {
        public final static double x = -POSITION_END_BLUE.x;
        public final static double y = -POSITION_END_BLUE.y;
        public final static double heading = normalizeAngle(POSITION_END_BLUE.heading + Math.PI);
    }

    static class POSITION_SCORING_TOP_BLUE {
        public final static double x = 9;
        public final static double y = 72 - 9;
        public final static double heading = Math.PI / 2;
    }

    static class POSITION_SCORING_BOTTOM_BLUE {
        public final static double x = 12;
        public final static double y = -72 + 9;
        public final static double heading = (3 * Math.PI) / 2;
    }

    static class POSITION_SCORING_TOP_RED {
        public final static double x = -POSITION_SCORING_TOP_BLUE.x;
        public final static double y = -POSITION_SCORING_TOP_BLUE.y;
        public final static double heading = normalizeAngle(POSITION_SCORING_TOP_BLUE.heading + Math.PI);
    }

    static class POSITION_SCORING_BOTTOM_RED {
        public final static double x = -POSITION_SCORING_BOTTOM_BLUE.x;
        public final static double y = -POSITION_SCORING_BOTTOM_BLUE.y;
        public final static double heading = normalizeAngle(POSITION_SCORING_BOTTOM_BLUE.heading + Math.PI);
    }

    public enum START_POS {
        BLUE, RED
    }

    public enum GOAL_POS {
        NEAR, FAR
    }

    // Proportional gains: power per inch of distance error, power per radian of heading error
    public static double KP_DRIVE = 0.02;
    public static double KP_TURN = 0.30;

    // Mecanum wheels strafe less efficiently than they drive forward. Values > 1.0 boost the
    // sideways share of the drive power so the robot travels in a straight line to the target.
    public static double STRAFE_MULTIPLIER = 1.35;

    // Drive (translation) power limits
    public static double MAX_DRIVE_POWER = 0.65;
    // Smallest drive power that still moves the robot (overcomes static friction)
    public static double MIN_DRIVE_POWER = 0.20;

    // Turn power limits
    public static double MAX_TURN_POWER = 0.2;
    // Smallest turn power that still rotates the robot, so small heading errors don't stall
    public static double MIN_TURN_POWER = 0.14;
    // Share of wheel capacity kept for turning while driving, so the turn is spread across the move
    // instead of being squeezed out by the drive and done all at once near the target.
    public static double TURN_RESERVE = 0.3;

    // Stall recovery: if the robot hasn't moved STALL_MOVE_INCHES or turned STALL_TURN_RADIANS for
    // STALL_TIME_SECONDS while not yet at the target, drive and turn power rise by
    // STALL_BOOST_PER_SECOND (up to MAX_STALL_BOOST) until it moves again.
    public static double STALL_TIME_SECONDS = 0.3;
    public static double STALL_MOVE_INCHES = 0.25;
    public static double STALL_TURN_RADIANS = Math.toRadians(1.0);
    public static double STALL_BOOST_PER_SECOND = 0.3;
    public static double MAX_STALL_BOOST = 0.4;
    // Heading errors smaller than this get no turn correction
    public static double TURN_DEADBAND_RADIANS = Math.toRadians(2.0);

    // Tolerances
    public static double POSITION_TOLERANCE_INCHES = 1.0;
    public static double HEADING_TOLERANCE_RADIANS = Math.toRadians(2.0);

    // Motor references
    private final DcMotor frontLeft;
    private final DcMotor backLeft;
    private final DcMotor frontRight;
    private final DcMotor backRight;

    Telemetry telemetry;

    private GoBildaPinpointDriver pinpoint;

    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetHeading = 0.0; //radians

    private double currentX = 0.0;
    private double currentY = 0.0;
    private double currentHeading = 0.0; //radians

    //red or blue starting position coordinates and heading
    private double startX = 0.0;
    private double startY = 0.0;
    private double startHeading = 0.0; //radians

    private double endX = 0.0;
    private double endY = 0.0;
    private double endHeading = 0.0; //radians

    private double scoreX = 0.0;
    private double scoreY = 0.0;
    private double scoreHeading = 0.0; //radians

    private List<FieldObstaclePlanner.Waypoint> currentPath = new ArrayList<>();
    private int waypointIndex = 0;
    // Set once the robot first gets inside the position tolerance of the final waypoint. From then on
    // only the heading is finished, in place, so drift from turning can't restart position correction.
    private boolean finalPositionSettled = false;
    // Drive power limit for the current move. Every new move starts at MAX_DRIVE_POWER; the
    // setTargetPositionAndPath overload with maxPower lowers it for that one move only.
    private double moveMaxPower = MAX_DRIVE_POWER;

    // Stall tracking: pose when the robot last moved, when that was (-1 = not started), current boost
    private double stallRefX, stallRefY, stallRefHeading;
    private double stallStartSec = -1;
    private double stallBoost = 0.0;

    // Drive-through move (setDriveThroughTarget): constant power, no slow-down, finishes when the
    // robot reaches or passes the target. Start point defines the direction of travel.
    private boolean driveThrough = false;
    private double driveThroughPower = 0.0;
    private double driveThroughStartX, driveThroughStartY;
    // Last place the drive-through move made progress, and when (-1 = not started)
    private double driveThroughRefX, driveThroughRefY;
    private double driveThroughRefSec = -1;

    private START_POS startPos = null;
    private GOAL_POS goalPosition = GOAL_POS.NEAR;

    private boolean firstPreference;

    public GoToPositionController(DcMotor fl, DcMotor bl, DcMotor fr, DcMotor br, GoBildaPinpointDriver pp, Telemetry tel, START_POS sp, boolean fp) {
        this.frontLeft = fl;
        this.backLeft = bl;
        this.frontRight = fr;
        this.backRight = br;
        this.pinpoint = pp;
        this.telemetry = tel;

        this.firstPreference = fp;

        startPos = sp;
        setStartPos();

        currentX = startX;
        currentY = startY;
        currentHeading = startHeading;

        initPinpoint();

        setScoringPos();
        setEndPos();

        telemetry.addData("Setting Home (X, Y, Heading)", "(%.1f, %.1f, %.1f radians)",
                startX, startY, startHeading);

        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                startX,
                startY,
                AngleUnit.RADIANS,
                startHeading
        ));


    }

    public void setGoalPosition (GOAL_POS gp) {

            goalPosition = gp;
            setScoringPos();
            targetX = scoreX;
            targetY = scoreY;
            targetHeading = scoreHeading;
            navigateObstacles();

    }

    public String getGoalPosition() {
        return goalPosition.toString();
    }

    /**
     * Drives one step toward the active waypoint. Callers must call odometry() once per loop
     * before calling this so the pose is current.
     */
    public boolean autoAssistDrive() {

        boolean destinationReached = false;

        if (currentPath == null || currentPath.isEmpty() || waypointIndex >= currentPath.size()) {
            stopMotors();
            return true;
        }

        FieldObstaclePlanner.Waypoint activeWaypoint = currentPath.get(waypointIndex);

        boolean reachedWaypoint = update(
                currentX,
                currentY,
                currentHeading,
                activeWaypoint.x,
                activeWaypoint.y,
                activeWaypoint.heading,
                moveMaxPower
        );

        if (reachedWaypoint) {
            waypointIndex++;
            // If all waypoints (including final target) are reached, return to manual
            if (waypointIndex >= currentPath.size()) {
                stopMotors();
                destinationReached = true;
            }
        }

        return destinationReached;
    }

    public boolean update(double currentX, double currentY, double currentHeading,
                          double targetX, double targetY, double targetHeading,
                          double maxPower) {

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        double robotForward =  deltaX * Math.cos(currentHeading) + deltaY * Math.sin(currentHeading);
        double robotStrafe  = deltaX * Math.sin(currentHeading) - deltaY * Math.cos(currentHeading);
        double headingError = normalizeAngle(targetHeading - currentHeading);
        double linearDistance = Math.hypot(deltaX, deltaY);

        boolean isFinalWaypoint = (waypointIndex >= currentPath.size() - 1);

        // Drive-through move: done as soon as the robot reaches or passes the target along its line
        // of travel. Brake there; no position tolerance, no final heading check, no slow-down.
        if (driveThrough) {
            double pathX = targetX - driveThroughStartX;
            double pathY = targetY - driveThroughStartY;
            double pathLength = Math.hypot(pathX, pathY);
            double along = (pathLength > 1e-6)
                    ? ((currentX - driveThroughStartX) * pathX + (currentY - driveThroughStartY) * pathY) / pathLength
                    : 0.0;
            if (pathLength < 1e-6 || along >= pathLength) {
                stopMotors();
                return true;
            }

            // Blocked (for example an artifact jammed against the wall): if the robot hasn't moved
            // STALL_MOVE_INCHES in STALL_TIME_SECONDS, stop pushing and end the move.
            double dtNowSec = System.nanoTime() / 1e9;
            if (driveThroughRefSec < 0
                    || Math.hypot(currentX - driveThroughRefX, currentY - driveThroughRefY) > STALL_MOVE_INCHES) {
                driveThroughRefX = currentX;
                driveThroughRefY = currentY;
                driveThroughRefSec = dtNowSec;
            } else if (dtNowSec - driveThroughRefSec > STALL_TIME_SECONDS) {
                stopMotors();
                return true;
            }
        }

        boolean positionReached = !driveThrough && linearDistance < POSITION_TOLERANCE_INCHES;
        if (isFinalWaypoint && positionReached) {
            finalPositionSettled = true;
        }
        if (isFinalWaypoint && finalPositionSettled) {
            positionReached = true;
        }
        boolean headingReached = !isFinalWaypoint || (Math.abs(headingError) < HEADING_TOLERANCE_RADIANS);

        if (positionReached && headingReached) {
            if (isFinalWaypoint) {
                stopMotors();
            }
            return true;
        }

        // Stall detection: restart the clock whenever the robot moves or turns; once it has been
        // still for STALL_TIME_SECONDS, ramp up stallBoost.
        double nowSec = System.nanoTime() / 1e9;
        boolean moved = Math.hypot(currentX - stallRefX, currentY - stallRefY) > STALL_MOVE_INCHES
                || Math.abs(normalizeAngle(currentHeading - stallRefHeading)) > STALL_TURN_RADIANS;
        if (stallStartSec < 0 || moved) {
            stallRefX = currentX;
            stallRefY = currentY;
            stallRefHeading = currentHeading;
            stallStartSec = nowSec;
            stallBoost = 0.0;
        } else if (nowSec - stallStartSec > STALL_TIME_SECONDS) {
            stallBoost = Math.min(MAX_STALL_BOOST,
                    (nowSec - stallStartSec - STALL_TIME_SECONDS) * STALL_BOOST_PER_SECOND);
        }

        // 1. Calculate turn power clamped to allowable range
        // Pinpoint heading is counter-clockwise positive, but positive turnPower in the wheel mix
        // below turns clockwise (same as right stick right in manual drive), so negate the error.
        double turnPower = 0.0;
        if (Math.abs(headingError) > TURN_DEADBAND_RADIANS) {
            turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, -headingError * KP_TURN));
            // Floor only applies when turning in place; while driving the wheels are already moving
            // and the floor over-corrects, causing heading wiggle that fights the translation.
            if (positionReached && Math.abs(turnPower) < MIN_TURN_POWER) {
                turnPower = Math.copySign(MIN_TURN_POWER, turnPower);
            }
            // Stall boost, still within the turn limit
            turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER,
                    turnPower + Math.copySign(stallBoost, turnPower)));
        }
        // Maintain a floor above friction, proportionally scaling down to target
        double driveMagnitude = Math.min(maxPower, Math.max(MIN_DRIVE_POWER, linearDistance * KP_DRIVE));
        // Stall boost, still within this move's drive limit
        driveMagnitude = Math.min(maxPower, driveMagnitude + stallBoost);
        // Drive-through move: constant power all the way, no slow-down near the target
        if (driveThrough) {
            driveMagnitude = driveThroughPower;
        }

        double forwardPower = 0.0;
        double strafePower = 0.0;
        if (!positionReached && linearDistance > 1e-4) {
            forwardPower = (robotForward / linearDistance) * driveMagnitude;
            strafePower  = (robotStrafe  / linearDistance) * driveMagnitude * STRAFE_MULTIPLIER;
        }

        // Share wheel capacity between drive and turn so both happen together for the whole move:
        // the turn always keeps up to TURN_RESERVE (only as much as it needs), the drive gets the rest
        // in full, and the turn is limited to what is left so the drive is never scaled down further.
        double translationLoad = Math.abs(forwardPower) + Math.abs(strafePower);
        double maxTranslation = 1.0 - Math.min(TURN_RESERVE, Math.abs(turnPower));
        if (translationLoad > maxTranslation) {
            double scale = maxTranslation / translationLoad;
            forwardPower *= scale;
            strafePower *= scale;
            translationLoad = maxTranslation;
        }
        double turnHeadroom = Math.max(0.0, 1.0 - translationLoad);
        turnPower = Math.max(-turnHeadroom, Math.min(turnHeadroom, turnPower));

        // 4. Combine into wheel powers
        double flPower = forwardPower + strafePower + turnPower;
        double blPower = forwardPower - strafePower + turnPower;
        double frPower = forwardPower - strafePower - turnPower;
        double brPower = forwardPower + strafePower - turnPower;

        setMotorPowers(flPower, blPower, frPower, brPower);

        telemetry.addData("Start Pos", startPos);
        telemetry.addData("Scoring Pos", goalPosition);
        telemetry.addData("Current (X, Y, Heading)", "(%.1f, %.1f, %.1f radians)",
                currentX, currentY, currentHeading);
        telemetry.addData("Current path segment", waypointIndex);
        telemetry.addData("Distance Error", linearDistance);
        telemetry.addData("Heading Error Deg", Math.toDegrees(headingError));

        return false;
    }

    /**
     * Desaturates and sets power to all four drive motors.
     */
    // Last powers sent to the drive motors, for logState()
    private double lastFl, lastBl, lastFr, lastBr;

    private void setMotorPowers(double fl, double bl, double fr, double br) {
        double maxMagnitude = Math.max(Math.abs(fl),
                              Math.max(Math.abs(bl),
                              Math.max(Math.abs(fr), Math.abs(br))));

        if (maxMagnitude > 1.0) {
            fl /= maxMagnitude;
            bl /= maxMagnitude;
            fr /= maxMagnitude;
            br /= maxMagnitude;
        }

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
        lastFl = fl;
        lastBl = bl;
        lastFr = fr;
        lastBr = br;
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        lastFl = 0;
        lastBl = 0;
        lastFr = 0;
        lastBr = 0;
    }

    /**
     * Writes the current pose, active waypoint and last drive powers to the Robot Controller log
     * (tag "SC2627"). Unlike telemetry, the log survives the OpMode being stopped.
     */
    public void logState(String label) {
        String waypoint = "none";
        if (currentPath != null && waypointIndex < currentPath.size()) {
            FieldObstaclePlanner.Waypoint w = currentPath.get(waypointIndex);
            waypoint = String.format("%d (%.1f, %.1f, %.1f deg)", waypointIndex, w.x, w.y, Math.toDegrees(w.heading));
        }
        RobotLog.ii("SC2627", "%s t=%.3f pose=(%.1f, %.1f, %.1f deg) waypoint=%s powers fl=%.2f bl=%.2f fr=%.2f br=%.2f boost=%.2f",
                label, System.nanoTime() / 1e9, currentX, currentY, Math.toDegrees(currentHeading), waypoint,
                lastFl, lastBl, lastFr, lastBr, stallBoost);
    }

    public static double normalizeAngle(double radians) {
        while (radians > Math.PI)  radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }

    protected void initPinpoint() {
                /*
        X pod offset:
         - refers to how far sideways from the tracking point the X (forward) odometry pod is
         - left of the center is a positive number, right of center is a negative number
        Y pod offset:
         - refers to how far forwards from the tracking point the Y (strafe) odometry pod is
         - forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-70, -22.0, DistanceUnit.MM);
        // Calibrated 4-bar pod resolution (nominal 19.894 ticks/mm). Hand-push over 96": forward read 94,
        // sideways read 92. One value applies to both pods, so this averages the two corrections
        // (19.48 forward, 19.07 sideways); each axis now reads within about 1%.
        pinpoint.setEncoderResolution(19.27, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //pinpoint.resetPosAndIMU();
    }

    /** Sets the Pinpoint (and this controller's copy) to a known field pose: inches, inches, radians. */
    /** True if the last odometry() read came from a Pinpoint reporting READY (not calibrating or a bad read). */
    public boolean isPinpointReady() {
        return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public void setPose(double x, double y, double heading) {
        Pose2D pose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading);

        // Wait (up to 1 s) for the Pinpoint to report READY; a write before then can be lost.
        long readyDeadline = System.nanoTime() + 1_000_000_000L;
        while (System.nanoTime() < readyDeadline) {
            pinpoint.update();
            if (isPinpointReady()) {
                break;
            }
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        // The constructor has just written the start pose to the Pinpoint. Write the new pose, then
        // read it back and write again until the Pinpoint reports it, so the start pose can't win.
        for (int attempt = 0; attempt < 10; attempt++) {
            pinpoint.setPosition(pose);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
            pinpoint.update();
            Pose2D reported = pinpoint.getPosition();
            if (Math.hypot(reported.getX(DistanceUnit.INCH) - x, reported.getY(DistanceUnit.INCH) - y) < 0.5
                    && Math.abs(normalizeAngle(reported.getHeading(AngleUnit.RADIANS) - heading)) < Math.toRadians(1.0)) {
                break;
            }
        }
        Pose2D reported = pinpoint.getPosition();
        RobotLog.ii("SC2627", "setPose requested=(%.1f, %.1f, %.1f deg) pinpoint=(%.1f, %.1f, %.1f deg) status=%s",
                x, y, Math.toDegrees(heading),
                reported.getX(DistanceUnit.INCH), reported.getY(DistanceUnit.INCH),
                reported.getHeading(AngleUnit.DEGREES), pinpoint.getDeviceStatus());

        currentX = x;
        currentY = y;
        currentHeading = heading;
    }

    // Pose from the most recent odometry() call (inches, inches, radians)
    public double getX() { return currentX; }
    public double getY() { return currentY; }
    public double getHeading() { return currentHeading; }

    public void odometry() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        currentX = pose.getX(DistanceUnit.INCH);
        currentY = pose.getY(DistanceUnit.INCH);
        currentHeading = pose.getHeading(AngleUnit.RADIANS);
    }

    private void setScoringPos() {
        // 1. Read Candidate A using your existing method
        if (startPos == START_POS.BLUE) {
            setTopBlueScoringPos();
        } else {
            setBottomRedScoringPos();
        }
        double posAX = scoreX;
        double posAY = scoreY;
        double posAHeading = scoreHeading;

        // 2. Read Candidate B using your existing method
        if (startPos == START_POS.BLUE) {
            setBottomBlueScoringPos();
        } else {
            setTopRedScoringPos();
        }
        double posBX = scoreX;
        double posBY = scoreY;
        double posBHeading = scoreHeading;

        // 3. Compute distance to each candidate
        double distA = Math.hypot(posAX - currentX, posAY - currentY);
        double distB = Math.hypot(posBX - currentX, posBY - currentY);

        // 4. Assign the winner back to scoreX, scoreY, scoreHeading
        boolean chooseA = (goalPosition == GOAL_POS.NEAR) ? (distA <= distB) : (distA >= distB);

        if (chooseA) {
            scoreX = posAX;
            scoreY = posAY;
            scoreHeading = posAHeading;
        } else {
            scoreX = posBX;
            scoreY = posBY;
            scoreHeading = posBHeading;
        }
    }

    private void setStartPos() {
        if (startPos == START_POS.BLUE ) {
            setBlueStart();
        } else {
            setRedStart();
        }
    }

    private void setEndPos() {
        if (startPos == START_POS.BLUE ) {
            setBlueEnd();
        } else {
            setRedEnd();
        }
    }

    public void goEnd() {
        targetX = endX;
        targetY = endY;
        targetHeading = endHeading;
        navigateObstacles();
    }

    public void goScore(GOAL_POS pos) {
        goalPosition = pos;
        setScoringPos();
        targetX = scoreX;
        targetY = scoreY;
        targetHeading = scoreHeading;
        navigateObstacles();
    }

    public void setTargetPositionAndPath(double x, double y, double heading) {
        targetX = x;
        targetY = y;
        targetHeading = heading;
        currentPath.clear();
        currentPath.add(new FieldObstaclePlanner.Waypoint(targetX, targetY, targetHeading));
        waypointIndex = 0;
        finalPositionSettled = false;
        moveMaxPower = MAX_DRIVE_POWER;
        stallStartSec = -1;
        stallBoost = 0.0;
        driveThrough = false;
    }

    /**
     * Drive-through move: drives toward (x, y) at a constant power (0 to 1.0) while turning to heading,
     * without slowing down near the end, and finishes as soon as the robot reaches or passes the point.
     * The motors then brake, so the robot stops a little past (x, y). Use for intake runs that need speed.
     */
    public void setDriveThroughTarget(double x, double y, double heading, double power) {
        setTargetPositionAndPath(x, y, heading);
        driveThrough = true;
        driveThroughPower = Math.max(0.0, Math.min(1.0, power));
        driveThroughStartX = currentX;
        driveThroughStartY = currentY;
        driveThroughRefSec = -1;
    }

    /** Same as setTargetPositionAndPath(x, y, heading), but drives this one move at no more than maxPower. */
    public void setTargetPositionAndPath(double x, double y, double heading, double maxPower) {
        setTargetPositionAndPath(x, y, heading);
        moveMaxPower = Math.min(maxPower, MAX_DRIVE_POWER);
    }

    /** Like setTargetPositionAndPath(x, y, heading), but routes around the hazard rectangles. */
    public void setTargetAndPlanPath(double x, double y, double heading) {
        targetX = x;
        targetY = y;
        targetHeading = heading;
        navigateObstacles();
    }

    private void navigateObstacles() {
          currentPath = FieldObstaclePlanner.planPath(currentX, currentY, targetX, targetY, targetHeading);
        waypointIndex = 0;
        finalPositionSettled = false;
        moveMaxPower = MAX_DRIVE_POWER;
        stallStartSec = -1;
        stallBoost = 0.0;
        driveThrough = false;

    }

    //HOME
    private void setBlueStart() {
        if (firstPreference) {
            startX = POSITION_START_BLUE_FIRST_PREFERENCE.x;
            startY = POSITION_START_BLUE_FIRST_PREFERENCE.y;
            startHeading = POSITION_START_BLUE_FIRST_PREFERENCE.heading;
        } else {
            startX = POSITION_START_BLUE_SECOND_PREFERENCE.x;
            startY = POSITION_START_BLUE_SECOND_PREFERENCE.y;
            startHeading = POSITION_START_BLUE_SECOND_PREFERENCE.heading;
        }
    }

    private void setRedStart() {
        if (firstPreference) {
            startX = POSITION_START_RED_FIRST_PREFERENCE.x;
            startY = POSITION_START_RED_FIRST_PREFERENCE.y;
            startHeading = POSITION_START_RED_FIRST_PREFERENCE.heading;
        } else {
            startX = POSITION_START_RED_SECOND_PREFERENCE.x;
            startY = POSITION_START_RED_SECOND_PREFERENCE.y;
            startHeading = POSITION_START_RED_SECOND_PREFERENCE.heading;
        }
    }

    //END
    private void setBlueEnd() {
        endX = POSITION_END_BLUE.x;
        endY = POSITION_END_BLUE.y;
        endHeading = POSITION_END_BLUE.heading;
    }

    private void setRedEnd() {
        endX = POSITION_END_RED.x;
        endY = POSITION_END_RED.y;
        endHeading = POSITION_END_RED.heading;
    }

    //NEAR SCORING
    private void setTopBlueScoringPos() {
        scoreX = POSITION_SCORING_TOP_BLUE.x;
        scoreY = POSITION_SCORING_TOP_BLUE.y;
        scoreHeading = POSITION_SCORING_TOP_BLUE.heading;
    }

    private void setBottomRedScoringPos() {
        scoreX = POSITION_SCORING_BOTTOM_RED.x;
        scoreY = POSITION_SCORING_BOTTOM_RED.y;
        scoreHeading = POSITION_SCORING_BOTTOM_RED.heading;
    }

    //FAR SCORING
    private void setBottomBlueScoringPos() {
        scoreX = POSITION_SCORING_BOTTOM_BLUE.x;
        scoreY = POSITION_SCORING_BOTTOM_BLUE.y;
        scoreHeading = POSITION_SCORING_BOTTOM_BLUE.heading;
    }

    private void setTopRedScoringPos() {
        scoreX = POSITION_SCORING_TOP_RED.x;
        scoreY = POSITION_SCORING_TOP_RED.y;
        scoreHeading = POSITION_SCORING_TOP_RED.heading;
    }

    public double calculateTotalPathLength() {
        if (currentPath == null || currentPath.size() < 2) {
            return 0.0;
        }

        double totalDistance = 0.0;
        for (int i = 0; i < currentPath.size() - 1; i++) {
            FieldObstaclePlanner.Waypoint p1 = currentPath.get(i);
            FieldObstaclePlanner.Waypoint p2 = currentPath.get(i + 1);
            totalDistance += Math.hypot(p2.x - p1.x, p2.y - p1.y);
        }

        return totalDistance;
    }

    public void addTelemetry() {
        telemetry.addData("Current (X, Y, Heading)", "(%.1f, %.1f, %.1f deg)",
                currentX, currentY, Math.toDegrees(currentHeading));
        telemetry.addData("Target (X, Y, Heading)", "(%.1f, %.1f, %.1f deg)",
                targetX, targetY, Math.toDegrees(targetHeading));
    }

}