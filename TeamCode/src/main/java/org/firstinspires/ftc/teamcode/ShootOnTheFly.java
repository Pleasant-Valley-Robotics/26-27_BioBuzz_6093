package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class ShootOnTheFly extends OpMode {

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

        Pose adjustedTarget = getMovingShotTarget(follower.pose(), follower.velocity());

        double adjustedVelocity = getBallVelocity(calculateDistance(follower.pose(), adjustedTarget));
        double adjustedAngle = Math.atan2(adjustedTarget.y(), adjustedTarget.x());

    }

    public double getLaunchRPM() {
        return 0;
    }

    Pose getMovingShotTarget(Pose robotPose, Velocity robotVelocity) {
        // Start guessing at a true  target position
        double virtualX = GOAL_POSE.x() - (robotVelocity.vx * time);
        double virtualY = GOAL_POSE.y() - (robotVelocity.vy * time);

        double distance = calculateDistance(robotPose, new Pose(virtualX, virtualY));
        double launchVelocity = getBallVelocity(distance);
        double time = calculateTime(distance, launchVelocity);

        // Run three times to lock in on actual position
        for (int i = 0; i < 3; i++) {
            // Redifine virtual x with our new time
            virtualX = GOAL_POSE.x() - (robotVelocity.vx * time);
            virtualY = GOAL_POSE.y() - (robotVelocity.vy * time);

            // Re-measure distance from physical robot to the new virtual target
            distance = calculateDistance(robotPose, new Pose(virtualX, virtualY));
            launchVelocity = getBallVelocity(distance);
            time = calculateTime(distance, launchVelocity);
        }

        return new Pose(virtualX, virtualY);
    }

    public double getBallVelocity(double distance) {
        return (distance / Math.cos(LAUNCH_ANGLE))
                * Math.sqrt(GRAVITY /
                (2 * (distance * Math.tan(LAUNCH_ANGLE) - (GOAL_HEIGHT - LAUNCH_HEIGHT))));
    }

    public double calculateDistance(Pose p1, Pose p2) {
        return Math.sqrt(Math.pow(p1.x() - p2.x(), 2) + Math.pow(p1.y() - p2.y(), 2));
    }

    public double calculateTime(double distance, double velocity) {
        return distance  / (velocity * Math.cos(LAUNCH_ANGLE));
    }
}
