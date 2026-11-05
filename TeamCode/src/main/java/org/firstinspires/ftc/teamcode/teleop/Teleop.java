package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="Testing teleop with changeable speed")
public class Teleop extends OpMode {
    private Follower follower;
    private DcMotorEx shootMotor;
    private DcMotorEx intakeMotor;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    private Servo light;
    private IMU imu;

    private int motorSpeed = 1275;
    private boolean shooterOn = false;

    public void init() {
        follower = Constants.create(hardwareMap);

        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        light = hardwareMap.get(Servo.class, "light");

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        // Code here runs as fast as possible until you pres stop on the control hub
        // Here you would get button presses, calculate motor power, read sensor data, etc.
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        if (gamepad2.right_trigger_pressed || gamepad2.left_trigger_pressed) {
            if (gamepad2.right_trigger_pressed) {
                windmillServo.setPower(-.15);
            } else {
                windmillServo.setPower(.15);
            }
        } else {
            windmillServo.setPower(gamepad2.right_stick_y);
        }


        if (gamepad2.dpadUpWasPressed()) {
            motorSpeed += 50;
        }

        if (gamepad2.dpadDownWasPressed()) {
            motorSpeed -= 50;
        }

        intakeMotor.setPower(gamepad2.left_stick_y);
        leftIntakeServo.setPower(gamepad2.left_stick_y);
        rightIntakeServo.setPower(gamepad2.left_stick_y);

        if (gamepad2.aWasPressed()) {
            shooterOn = !shooterOn;
        }

        if (shooterOn) {
            shootMotor.setVelocity(motorSpeed);
            if (Math.abs(shootMotor.getVelocity() - motorSpeed) < 30) {
                light.setPosition(.5);
            } else {
                light.setPosition(.278);
            }
        } else {
            shootMotor.setPower(0);
            light.setPosition(0);
        }

        if (gamepad1.left_trigger_pressed) {
            forward *= .35;
            strafe *= .35;
            rotate *= .35;
        }
        if (gamepad1.right_trigger_pressed) {
            ManualDrive.driveOrHold(follower, forward, strafe, rotate);
        } else {
            driveFieldRelative(forward, strafe, rotate);
        }

        telemetry.addData("Target TPS", motorSpeed);
        telemetry.addData("shooterMotor TPS", shootMotor.getVelocity());
        telemetry.addData("shooterMotor RPM", (shootMotor.getVelocity()/26)*60);
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.update();

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
}
