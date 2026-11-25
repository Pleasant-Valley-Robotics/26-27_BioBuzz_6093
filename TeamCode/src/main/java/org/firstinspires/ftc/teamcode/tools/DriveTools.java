package org.firstinspires.ftc.teamcode.tools;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Configurable
public class DriveTools {
    // Mostly tools for auto locking
    public static final Pose autolockingTargetB = new Pose(57.889, 53.442);
    public static final Pose autolockingTargetF = new Pose(57.59, 88.058);
    public static Pose autoLockingTarget = autolockingTargetB;
    public static double kPLimelight = .05;
    public static double kPOdometry = 1.0;

    public static Pose chooseTarget (Follower follower) {
        if (follower.pose().y() > 70.75) {
            // If we are past the center point, set the autoLocking target to far
            autoLockingTarget = autolockingTargetF;
        } else {
            // Otherwise set it to close
            autoLockingTarget = autolockingTargetB;
        }
        return autoLockingTarget; // Return the target for telemetry
    }
    public static double findAutoLockingAngle(Follower follower) {
        // Uses trig to find the angle we need to aim at
        // We have an x and a y, we can use tangent to find our target heading

        double xDif = autoLockingTarget.x() - follower.pose().x();
        double yDif = autoLockingTarget.y() - follower.pose().y();

        // Use tan-1 to get raw target heading before normalizing
        double rawTargetHeading = Math.atan2(yDif, xDif);
        // Normalize radians within -2(pi), 2(pi)
        // This shouldn't really be necessary as we normalize again later, but it doesn't hurt anything
        rawTargetHeading = AngleUnit.normalizeRadians(rawTargetHeading);
        return AngleUnit.normalizeRadians(Math.PI + rawTargetHeading);
    }

    public static double findAutoLockPower(Follower follower, Vision vision, double targetHeading) {
        double visionError = vision.findErrorForAutoLocking(Storage.color, (follower.pose().y() > 70.75));
        if (visionError != -999999) { // If we have a vision error, use this first
            double turnPower = kPLimelight * visionError;

            return Math.max(-.2, Math.min(.2, turnPower)); // Set max turning speed - this is low for vision auto locking
        }

        double tolerance = 0.0; // Tolerance in radians - this is set to zero because we actually just don't care

        // If we can't use vision, just use odometry instead
        double error = targetHeading - follower.pose().heading();
        // Normalize within -2(pi), 2(pi) so it doesn't try to spin multiple times
        error = AngleUnit.normalizeRadians(error);

        if (Math.abs(error) > tolerance) {
            // Only run while error is outside of tolerance
            double turnPower = kPOdometry * error;


            return Math.max(-.8, Math.min(.8, turnPower)); // Set max turning speed
        } else {
            // We are aligned, so command no turn.
            return 0.0;
        }
    }

    // Unused as of now, but copied from last year
    public static void driveFieldRelative(Follower follower, double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);



        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                follower.pose().heading());

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        follower.manual(newForward, -newRight, rotate); // CHANGE THIS BACK TO DRIVE OR HOLD
    }
}
