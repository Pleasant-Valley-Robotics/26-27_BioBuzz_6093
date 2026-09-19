package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class DriveWhileShoot extends OpMode {

    private final double GOAL_HEIGHT = 3; // Feet
    private final double GRAVITY = 32.1740; // Feet
    private final double LAUNCH_HEIGHT = 1; // Feet
    private final double LAUNCH_ANGLE = Math.toRadians(55);
    private final Pose GOAL_POSE = new Pose(0, 0);

    private Follower follower;



    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    public void loop() {
        // Drive or hold automatically tries to resist movement when sticks are idle
        ManualDrive.driveOrHold(follower,
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);

        // Update the follower to get a new position
        follower.update();

        Pose adjustedTarget = getFinalEffectiveTarget(follower.pose(), follower.velocity());

        double adjustedVelocity = getBallVelocity(adjustedTarget);
        double adjustedAngle = Math.atan2(adjustedTarget.y(), adjustedTarget.x());

    }

    public double getLaunchRPM() {
        return 0;
    }

    // This is such a terrible method name, feel free to rename it to something better
    Pose getFinalEffectiveTarget(Pose robotPose, Velocity robotVelocity) {
        // Find launch velocity you would use if your robot is completely stationary
        double launchVelocity = getBallVelocity(robotPose);

        // Calculate time for shot using distance and velocity
        double time = calculateTime(calculateHorizontalDistance(robotPose), launchVelocity);

        // Calculate a new target position by multiplying velocity and time
        Pose effectiveTarget = getEffectiveTarget(robotPose, robotVelocity, time);

        // Do the same thing 3 more times in order to lock in on an actual target position
        for (int i = 0; i < 4; i++) {
           launchVelocity = getBallVelocity(effectiveTarget);
           time = calculateTime(calculateHorizontalDistance(effectiveTarget), launchVelocity);
           effectiveTarget = getEffectiveTarget(effectiveTarget, robotVelocity, time);
        }

        return effectiveTarget;
    }

    public double getBallVelocity(Pose robotPose) {
        // Get straight line distance
        double x = calculateHorizontalDistance(robotPose);

        // Find velocity using particle motion equation
        return (x / Math.cos(LAUNCH_ANGLE))
                * Math.sqrt(GRAVITY / 2 * (x * Math.tan(LAUNCH_ANGLE)
                - GOAL_HEIGHT - LAUNCH_HEIGHT));
    }

    public double calculateHorizontalDistance(Pose robotPose) {
        // Calculates straight line distance using Pythagorean's Theorem
        return Math.sqrt(Math.pow(robotPose.x() - GOAL_POSE.x(), 2) + Math.pow(robotPose.y() - GOAL_POSE.y(), 2));
    }

    public double calculateTime(double x, double velocity) {
        // `
        return x / velocity * Math.cos(LAUNCH_HEIGHT);
    }

    public Pose getEffectiveTarget(Pose robotPose, Velocity robotVelocity, double time) {
        // Find the effective target position by multiplying velocity * time
        double effectiveX = robotPose.x() - (robotVelocity.vx * time);
        double effectiveY = robotPose.y() - (robotVelocity.vy * time);

        return new Pose(effectiveX, effectiveY);
    }
}
