package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@TeleOp(name="SOTF", group="aaaOdometry")
public class ShootOnTheFly extends OpMode {

    private final double GOAL_HEIGHT = 3; // Feet
    private final double GRAVITY = 32.1740; // Feet
    private final double LAUNCH_HEIGHT = .75; // Feet
    private final double LAUNCH_ANGLE = Math.toRadians(55);
    private final Pose GOAL_POSE = new Pose(11.8 / 12, 135.7 / 12); // x=129.7 for red
    private boolean motorOn = false;
    private boolean autoLocking = false;
    private Servo flickerServo;
    private DcMotorEx shooter;

    private Follower follower;
    private Turntable turnable;




    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0));
        turnable = new Turntable(hardwareMap);
        flickerServo = hardwareMap.get(Servo.class, "flicker");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    }

    public void loop() {
        // Drive or hold automatically tries to resist movement when sticks are idle
        double rotate;

        // Update the follower to get a new position
        follower.update();

        Pose robotPoseFeet = toFeet(follower.pose());
        Velocity robotVelocityFeet = toFeetPerSecond(follower.velocity());

        Pose adjustedTarget = getMovingShotTarget(robotPoseFeet, robotVelocityFeet);

        double adjustedVelocity = getBallVelocity(calculateDistance(robotPoseFeet, adjustedTarget));
        double motorSpeed = getLaunchTPS(adjustedVelocity);
        double adjustedAngle = Math.atan2(
                adjustedTarget.y() - robotPoseFeet.y(),
                adjustedTarget.x() - robotPoseFeet.x()
        );

        if (gamepad1.aWasPressed()) {motorOn = !motorOn;}
        if (gamepad1.xWasPressed()) {autoLocking = !autoLocking;}
        if (gamepad1.dpadRightWasPressed()) {turnable.turnRight();}
        if (gamepad1.dpadLeftWasPressed()) {turnable.turnLeft();}
        if (gamepad1.dpadUpWasPressed()) {
            flickerServo.setPosition( 0.38);}
        if (gamepad1.dpadDownWasPressed()) {flickerServo.setPosition(0.43);}

        if (motorOn) {shooter.setVelocity(motorSpeed);}
        if (autoLocking) {
            rotate = turnToAngle(adjustedAngle, follower.pose().heading());
        } else {
            rotate = -gamepad1.right_stick_x;
        }

        ManualDrive.driveOrHold(follower,
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                rotate);

        telemetry.addData("Adjusted Velocity", adjustedVelocity);
        telemetry.addData("Adjusted Angle", adjustedAngle);
        telemetry.addData("Motor Speed", motorSpeed);
        telemetry.addData("Current Position", follower.pose());
        telemetry.addData("Target Position", GOAL_POSE);

    }

    public double getLaunchTPS(double velocity) {
        double rpms =  (velocity * 120) / Math.PI * 0.314961;
        return (rpms / 60) * 28;
    }

    Pose getMovingShotTarget(Pose robotPose, Velocity robotVelocity) {
        double time = 1;
        // Start guessing at a true  target position
        double virtualX = GOAL_POSE.x() - (robotVelocity.vx * time);
        double virtualY = GOAL_POSE.y() - (robotVelocity.vy * time);

        double distance = calculateDistance(robotPose, new Pose(virtualX, virtualY));
        double launchVelocity = getBallVelocity(distance);
        time = calculateTime(distance, launchVelocity);

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


    public double turnToAngle(double targetHeading, double currentHeading) {
        // Use tangent to calculate the angle needed to face the goal position on the field
        double tolerance = Math.toRadians(1.5); // Tolerance in radians

        // Deviation is the error between our current heading and the calculated target heading
        double deviation = targetHeading - currentHeading;

        // Normalize within -2(pi), 2(pi) so it doesn't try to spin multiple times
        deviation = AngleUnit.normalizeRadians(deviation);


        if (Math.abs(deviation) > tolerance) {
            // Only run while error is outside of tolerance
            double kP = .8; // Modify to change P strength
            double turnPower = kP * deviation;


            return Math.max(-0.6, Math.min(0.6, turnPower)); // Set max turning speed
        } else {
            // We are aligned, so command no turn.
            return 0.0;
        }
    }

    public Pose toFeet(Pose inchPose) {
        return new Pose(inchPose.x() / 12.0, inchPose.y() / 12.0, inchPose.heading());
    }

    // Convert an inch-based Pedro Pathing Velocity vector to Feet per second
    public Velocity toFeetPerSecond(Velocity inchVelocity) {
        // Pedro Pathing's velocity is typically measured in inches per second
        return new Velocity(inchVelocity.vx / 12.0, inchVelocity.vy / 12.0, inchVelocity.omega);
    }
}
