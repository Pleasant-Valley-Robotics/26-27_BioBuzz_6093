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
}
