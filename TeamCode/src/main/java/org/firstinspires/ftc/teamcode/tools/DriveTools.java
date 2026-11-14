package org.firstinspires.ftc.teamcode.tools;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTools {
    public static final Pose autolockingTargetB = new Pose(57.59, 54.5);
    public static final Pose autolockingTargetF = new Pose(57.59, 87);
    public static Pose autoLockingTarget = autolockingTargetB;

    public static Pose chooseTarget (Follower follower) {
        if (follower.pose().y() > 70.75) {
            autoLockingTarget = autolockingTargetF;
        } else {
            autoLockingTarget = autolockingTargetB;
        }
        return autoLockingTarget;
    }
    public static double findAutoLockingAngle(Follower follower) {


        double xDif = autoLockingTarget.x() - follower.pose().x();
        double yDif = autoLockingTarget.y() - follower.pose().y();

        // Use tan-1 to get raw target heading before normalizing
        double rawTargetHeading = Math.atan2(yDif, xDif);
        // Normalize radians within -2(pi), 2(pi)
        // This shouldn't really be necessary as we normalize again later, but it doesn't hurt anything
        rawTargetHeading = AngleUnit.normalizeRadians(rawTargetHeading);
        return AngleUnit.normalizeRadians(Math.PI + rawTargetHeading);
    }

    public static double findAutoLockPower(Follower follower, double targetHeading) {
        double tolerance = 0.0; // Tolerance in radians

        double error = targetHeading - follower.pose().heading();
        // Normalize within -2(pi), 2(pi) so it doesn't try to spin multiple times
        error = AngleUnit.normalizeRadians(error);

        if (Math.abs(error) > tolerance) {
            // Only run while error is outside of tolerance
            double kP = 1.0; // Modify to change P strength
            double turnPower = kP * error;


            return Math.max(-0.6, Math.min(0.6, turnPower)); // Set max turning speed
        } else {
            // We are aligned, so command no turn.
            return 0.0;
        }
    }

    private void driveFieldRelative(Follower follower, IMU imu, double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);



        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        ManualDrive.driveOrHold(follower, newForward, newRight, rotate);
    }
}
