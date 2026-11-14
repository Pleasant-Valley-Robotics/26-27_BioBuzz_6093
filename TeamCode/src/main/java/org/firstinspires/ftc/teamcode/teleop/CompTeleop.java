package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.PanelsField;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@TeleOp(name="THIS ONE JESSE --->>>>> Teleop for comp")
public class CompTeleop extends OpMode {
    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private PanelsField panelsField = PanelsField.INSTANCE;
    private Follower follower;
    private Shooter shooter;
    private DcMotorEx intakeMotor;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    private IMU imu;

    private static int motorSpeed = 1275;
    private int shooterValue = 0;
    private boolean lightActive = false;
    private ElapsedTime runtime = new ElapsedTime();

    private boolean swapColors = true;
    private final Pose autolockingTargetB = new Pose(58, 50.75); //90.75 for opposite side
    private final Pose autolockingTargetF = new Pose(58, 90.75);
    private boolean autoLockingOn = false;

    public void init() {
        runtime.reset();
        follower = Constants.create(hardwareMap);
        shooter = new Shooter(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



    }

    public void loop() {
        // Code here runs as fast as possible until you pres stop on the control hub
        // Here you would get button presses, calculate motor power, read sensor data, etc.
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        if (gamepad2.right_trigger_pressed || gamepad2.left_trigger_pressed) {
            if (gamepad2.right_trigger_pressed) {
                windmillServo.setPower(0.5);
            } else {
                windmillServo.setPower(-0.5);
            }
        } else {
            windmillServo.setPower(gamepad2.right_stick_y);
        }

        intakeMotor.setPower(gamepad2.left_stick_y);
        leftIntakeServo.setPower(gamepad2.left_stick_y);
        rightIntakeServo.setPower(gamepad2.left_stick_y);

        if (gamepad2.xWasPressed()) {
            shooterValue = 1;
        }

        if (gamepad2.aWasPressed()) {
            shooterValue = 0;
        }

        if (gamepad2.bWasPressed()) {
            shooterValue = -1;
        }
        if (gamepad2.yWasPressed()) {
            swapColors = !swapColors;
        }

        if (shooterValue == 1) {
            shooter.setVelocity(motorSpeed);
            lightActive = true;
        } else if (shooterValue == -1) {
            shooter.setVelocity(-600);
            lightActive = true;
        } else {
            shooter.setVelocity(0);
            lightActive = false;
        }

        if (gamepad1.xWasPressed()) {
            autoLockingOn = !autoLockingOn;
        }


        if (gamepad1.bWasPressed()) {
            follower.setHeading(0);
        }

        if (gamepad1.left_trigger_pressed) {
            forward *= .35;
            strafe *= .35;
            rotate *= .35;
        }

        if (autoLockingOn) {
            if (gamepad1.right_trigger_pressed) {
                follower.hold(new Pose(follower.pose().x(), follower.pose().y(), findAutoLockingAngle()));
            } else {
                ManualDrive.driveOrHold(follower, forward, strafe,  autoLockPower(findAutoLockingAngle()));
            }
        } else {
            ManualDrive.driveOrHold(follower, forward, strafe, rotate);
        }

        shooter.updateLight(lightActive, swapColors);

        telemetry.addData("Runtime", runtime);
        telemetry.addData("Target TPS", motorSpeed);
        telemetry.addData("shooterMotor TPS", shooter.getVelocity());
        telemetry.addData("shooterMotor RPM", (shooter.getVelocity()/26)*60);
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Light on", swapColors);
        telemetry.addData("Position", follower.pose());
        telemetry.addData("Heading (degrees)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Rotate", rotate);
        telemetry.addData("Auto locking", autoLockingOn);
        telemetry.update();


        panelsTelemetry.getFtcTelemetry().addData("Target TPS", motorSpeed);
        panelsTelemetry.getFtcTelemetry().addData("Current TPS", shooter.getVelocity());
        panelsTelemetry.getFtcTelemetry().addData("shooterMotor RPM", (shooter.getVelocity()/26)*60);


        panelsTelemetry.getFtcTelemetry().update();


        follower.update();
    }


    private void driveFieldRelative(double forward, double right, double rotate) {
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

    private double findAutoLockingAngle() {
        Pose targetPose;
        if (follower.pose().y() > 70.75) {
            targetPose = autolockingTargetB;
        } else {
            targetPose = autolockingTargetF;
        }

        double xDif = Math.abs(targetPose.x() - follower.pose().x());
        double yDif = Math.abs(targetPose.y() - follower.pose().y());

        // Use tan-1 to get raw target heading before normalizing
        double rawTargetHeading = Math.atan2(yDif, xDif);
        // Normalize radians within -2(pi), 2(pi)
        // This shouldn't really be necessary as we normalize again later, but it doesn't hurt anything
        rawTargetHeading = AngleUnit.normalizeRadians(rawTargetHeading);
        return AngleUnit.normalizeRadians(rawTargetHeading);


    }

    private double autoLockPower(double targetHeading) {
        double tolerance = 0.0; // Tolerance in radians

        double error = targetHeading - follower.pose().heading();
        // Normalize within -2(pi), 2(pi) so it doesn't try to spin multiple times
        error = AngleUnit.normalizeRadians(error);

        if (Math.abs(error) > tolerance) {
            // Only run while error is outside of tolerance
            double kP = 1.0; // Modify to change P strength
            double turnPower = kP * error;
            telemetry.addData("turnPower",turnPower);


            return Math.max(-0.6, Math.min(0.6, turnPower)); // Set max turning speed
        } else {
            // We are aligned, so command no turn.
            return 0.0;
        }
    }
}
