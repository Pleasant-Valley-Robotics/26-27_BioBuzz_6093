package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.PanelsField;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.tools.DriveTools;
import org.firstinspires.ftc.teamcode.tools.MathTools;
import org.firstinspires.ftc.teamcode.tools.Storage;

// If you want Configurable to work correctly, you need to comment out slothLoad and autoTune.
// That's line 47 in the teamcode build.gradle file and line 19 in build.dependencies.gradle
@Configurable
@TeleOp(name="THIS ONE JESSE --->>>>> Teleop for comp")
public class CompTeleop extends OpMode {

    // Define our different subsystems - This has to be done here, otherwise they wouldn't be accessible in multiple functions at once
    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private IMU imu; // Imu was added for use in field centric, but Ethan decided that field centric sucked and removed it
    private Vision vision;

    // Panels Telemetry and Panels Field are both parts of the panels package. They use .INSTANCE because they were originally written in Kotlin
    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private final PanelsField panelsField = PanelsField.INSTANCE;
    private Pose holdPose = new Pose(0, 0); // Hold pose is used to hold the pose while auto locking, and should probably be local, not global
    private int shooterValue = 0; // Shooter value tells the shooter which way to spin


    private boolean lightActive = false;
    private boolean swapColors = true; // Swap colors changes the color the LED turns to when the shooter is up to speed


    private boolean autoLockingOn = false;


    private ElapsedTime runtime = new ElapsedTime(); // Tells us how long the opMode has been running



    public void init() {
        // Code here runs once when you press init

        // Set runtime to 0
        runtime.reset();
        // Define our subsystems by passing in the hardware map
        follower = Constants.create(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
    }

    public void loop() {
        // Code here runs as fast as possible until you press stop on the control hub
        // Here you would get button presses, calculate motor power, read sensor data, etc.
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        // Save distance to a variable so we don't have to call calculateDistance multiple times
        double distance =  MathTools.calculateDistance(follower.pose(), DriveTools.chooseTarget(follower));
        // Shoot speed uses distance to find the needed TPS to shoot at the goal
        //      Unfortunately this still isn't the most accurate because our windmill causes uncertainty in the way our pollen enters the shooter
        int shootSpeed = (int)Math.round(MathTools.calculateNeededTPS(distance));

        // This is pretty self explanatory, right trigger moves forward, left back
        if (gamepad2.right_trigger_pressed || gamepad2.left_trigger_pressed) {
            if (gamepad2.right_trigger_pressed) {
                shooter.windmillSetPower(1.0);
            } else {
                shooter.windmillSetPower(-1.0);
            }
        } else {
            shooter.windmillSetPower(0);
        }

        if (gamepad2.xWasPressed()) {shooterValue = 1;} // Shooter shoot forward
        if (gamepad2.aWasPressed()) {shooterValue = 0;} // Stop
        if (gamepad2.bWasPressed()) {shooterValue = -1;} // Reverse shooter
        if (gamepad2.yWasPressed()) {swapColors = !swapColors;}

        if (shooterValue == 1) {
            shooter.setVelocity(shootSpeed); // Set velocity targets a specific TPS, while .setPower() runs without encoders and uses straight voltage
            lightActive = true; // Turn on the light so it will update correctly
        } else if (shooterValue == -1) {
            shooter.setVelocity(-600);
            lightActive = true;
        } else {
            shooter.setVelocity(0); // Turn off the light so that our opponents don't get to much information
            lightActive = false;
        }

        intake.setPower(gamepad2.left_stick_y);


        if (gamepad1.xWasPressed()) {autoLockingOn = !autoLockingOn;}

        // This was measured in inches using fusion. It accounts for the size of our robot pressing against the wall
        if (gamepad1.bWasPressed()) {follower.setPose(new Pose(8.031, 8.89764, 0));}

        // This is a slow mode control, we scale each controller vector down
        if (gamepad1.left_trigger_pressed) {
            forward *= .35;
            strafe *= .35;
            rotate *= .35;
        }

        if (autoLockingOn) {
            // Right trigger tells the robot to hold in place.
            // This is pretty useless now, but it was added when we used field centric because pedro pathing didn't have a built in driveOrHold function for field centric
            if (gamepad1.right_trigger_pressed) {
                if (gamepad1.rightTriggerWasPressed()) {
                    holdPose = follower.pose();
                }
                // Again, all of this is pretty useless
                follower.hold(new Pose(holdPose.x(), holdPose.y(), DriveTools.findAutoLockingAngle(follower)));
            } else {
                // When auto locking on, we set the rotate vector of driveOrHold to be the power calculated from DriveTools
                ManualDrive.driveOrHold(follower, forward, strafe,  DriveTools.findAutoLockPower(follower, vision, DriveTools.findAutoLockingAngle(follower)));
            }
        } else {
            // Otherwise, just drive with the information from rotate (the gamepad input)
            ManualDrive.driveOrHold(follower, forward, strafe, rotate);
        }


        shooter.updateLight(lightActive, swapColors);

        // We have a bunch of telemetry, not all of it is useful, but in general its better to have too much information than not enough
        telemetry.addData("Runtime", runtime);
        telemetry.addData("Target TPS", shootSpeed);
        telemetry.addData("shooterMotor TPS", shooter.getVelocity());
        telemetry.addData("shooterMotor RPM", (shooter.getVelocity()/26)*60);
        telemetry.addData("Servo position", vision.autoFindServoPosition(distance));
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Light on", swapColors);
        telemetry.addData("Position", follower.pose());
        telemetry.addData("Heading (degrees)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Rotate", rotate);
        telemetry.addData("Auto locking", autoLockingOn);
        telemetry.addData("Distance to goal",distance);
        telemetry.addData("Auto locking target ", DriveTools.autoLockingTarget);
        telemetry.addData("Angle for auto locking", Math.toDegrees(DriveTools.findAutoLockingAngle(follower)));
        telemetry.addData("Needed velocity", MathTools.calculateNeededVelocity(distance));
        telemetry.update();

        // In order to have Panels graph your data, you have to use the format below
        panelsTelemetry.getFtcTelemetry().addData("Target TPS", shootSpeed);
        panelsTelemetry.getFtcTelemetry().addData("Current TPS", shooter.getVelocity());
        panelsTelemetry.getFtcTelemetry().addData("shooterMotor RPM", (shooter.getVelocity()/26)*60);
        panelsTelemetry.getFtcTelemetry().addData("Distance to goal",distance);
        panelsTelemetry.getFtcTelemetry().addData("Needed velocity", MathTools.calculateNeededVelocity(distance));
        panelsTelemetry.getFtcTelemetry().update();

        // Update our pose
        follower.update();
    }
}
