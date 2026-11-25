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

@Configurable
@TeleOp(name="THIS ONE JESSE --->>>>> Teleop for comp")
public class CompTeleop extends OpMode {

    private Follower follower;
    private Shooter shooter;
    private Intake intake;
    private IMU imu;
    private Vision vision;


    private final PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private final PanelsField panelsField = PanelsField.INSTANCE;
    private Pose holdPose = new Pose(0, 0);
    private int shooterValue = 0;


    private boolean lightActive = false;
    private boolean swapColors = true;


    private boolean autoLockingOn = false;


    private ElapsedTime runtime = new ElapsedTime();



    public void init() {
        runtime.reset();
        follower = Constants.create(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        vision = new Vision(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
    }

    public void loop() {
        // Code here runs as fast as possible until you pres stop on the control hub
        // Here you would get button presses, calculate motor power, read sensor data, etc.
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;


        double distance =  MathTools.calculateDistance(follower.pose(), DriveTools.chooseTarget(follower));
        int shootSpeed = (int)Math.round(MathTools.calculateNeededTPS(distance));

        if (gamepad2.right_trigger_pressed || gamepad2.left_trigger_pressed) {
            if (gamepad2.right_trigger_pressed) {
                shooter.windmillSetPower(1.0);
            } else {
                shooter.windmillSetPower(-1.0);
            }
        } else {
            shooter.windmillSetPower(0);
        }

        if (gamepad2.xWasPressed()) {shooterValue = 1;}
        if (gamepad2.aWasPressed()) {shooterValue = 0;}
        if (gamepad2.bWasPressed()) {shooterValue = -1;}
        if (gamepad2.yWasPressed()) {swapColors = !swapColors;}

        if (shooterValue == 1) {
            shooter.setVelocity(shootSpeed);
            lightActive = true;
        } else if (shooterValue == -1) {
            shooter.setVelocity(-600);
            lightActive = true;
        } else {
            shooter.setVelocity(0);
            lightActive = false;
        }

        intake.setPower(gamepad2.left_stick_y);


        if (gamepad1.xWasPressed()) {autoLockingOn = !autoLockingOn;}
        if (gamepad1.bWasPressed()) {follower.setPose(new Pose(8.031, 8.89764, 0));}

        if (gamepad1.left_trigger_pressed) {
            forward *= .35;
            strafe *= .35;
            rotate *= .35;
        }

        if (autoLockingOn) {
            if (gamepad1.right_trigger_pressed) {
                if (gamepad1.rightTriggerWasPressed()) {
                    holdPose = follower.pose();
                }
                follower.hold(new Pose(holdPose.x(), holdPose.y(), DriveTools.findAutoLockingAngle(follower)));
            } else {
                ManualDrive.driveOrHold(follower, forward, strafe,  DriveTools.findAutoLockPower(follower, vision, DriveTools.findAutoLockingAngle(follower)));
            }
        } else {
            ManualDrive.driveOrHold(follower, forward, strafe, rotate);
        }

        shooter.updateLight(lightActive, swapColors);


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


        panelsTelemetry.getFtcTelemetry().addData("Target TPS", shootSpeed);
        panelsTelemetry.getFtcTelemetry().addData("Current TPS", shooter.getVelocity());
        panelsTelemetry.getFtcTelemetry().addData("shooterMotor RPM", (shooter.getVelocity()/26)*60);
        panelsTelemetry.getFtcTelemetry().addData("Distance to goal",distance);
        panelsTelemetry.getFtcTelemetry().addData("Needed velocity", MathTools.calculateNeededVelocity(distance));
        panelsTelemetry.getFtcTelemetry().update();


        follower.update();
    }
}
