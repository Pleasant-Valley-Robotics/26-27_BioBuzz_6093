package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.math.Pose;

public class MathTools {
    private static final double LAUNCH_ANGLE = Math.toRadians(65.0); // IN RADIANS
    private static final double GOAL_HEIGHT = 59.55; // INCHES
    private static final double LAUNCH_HEIGHT = 7.0; // INCHES
    private static final double GRAVITY = 386.0886; // LITTLE G IN INCHES PER SECOND^2

    public static double calculateNeededVelocity(double distance) {
        return (distance / Math.cos(LAUNCH_ANGLE))
                * Math.sqrt(GRAVITY /
                (2 * (distance * Math.tan(LAUNCH_ANGLE) - (GOAL_HEIGHT - LAUNCH_HEIGHT))));
    }

    public static double calculateDistance(Pose p1, Pose p2) {
        return Math.sqrt(Math.pow(p1.x() - p2.x(), 2) + Math.pow(p1.y() - p2.y(), 2));
    }

    public static double calculateNeededTPS(double distance) {
        return (
                (7.269819343 * Math.pow(10, -5)) * Math.pow(distance, 4) +
                        (-0.0216330485) * Math.pow(distance, 3) +
                        (2.337016932) * Math.pow(distance, 2) +
                        (-104.6395725) * distance + 2949.474477
        );
    }
}
