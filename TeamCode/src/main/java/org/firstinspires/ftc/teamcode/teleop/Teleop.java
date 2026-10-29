package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="Basic TeleOp")
public class Teleop extends OpMode {
    private Follower follower;
    private DcMotorEx shootMotor;
    private DcMotorEx intakeMotor;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    private int motorSpeed = 1200;
    private boolean intakeOn = false;

    public void init() {
        follower = Constants.create(hardwareMap);

        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        if (gamepad1.right_trigger_pressed) {
            windmillServo.setPower(-0.25);
        }

        if (gamepad1.left_trigger_pressed) {
            windmillServo.setPower(0.0);
        }
        if (gamepad1.dpadUpWasPressed()) {
            motorSpeed += 50;
        }

        if (gamepad1.dpadDownWasPressed()) {
            motorSpeed -= 50;
        }

        if (gamepad1.a){
            intakeOn=!intakeOn;
        }

        if (gamepad1.b){
            intakeMotor.setPower(-1.0);
            leftIntakeServo.setPower(1.0);
            rightIntakeServo.setPower(1.0);
        } else if(intakeOn) {
            intakeMotor.setPower(1.0);
            leftIntakeServo.setPower(-1.0);
            rightIntakeServo.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
            leftIntakeServo.setPower(0.0);
            rightIntakeServo.setPower(0.0);
        }

        if (gamepad1.y) {
            shootMotor.setVelocity(motorSpeed);
        }

        if (gamepad1.x) {
            shootMotor.setPower(0.0);
        }

        ManualDrive.driveOrHold(follower, forward, strafe, rotate);

        telemetry.addData("Target TPS", motorSpeed);
        telemetry.addData("shooterMotor TPS", shootMotor.getVelocity());
        telemetry.addData("shooterMotor RPM", (shootMotor.getVelocity()/26)*60);
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.update();

        follower.update();
    }
}
