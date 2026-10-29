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
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
//
//        slowmode = gamepad1.right_trigger_pressed;
//
//        if (slowmode) {
//            forward *= 0.35;
//            strafe *= 0.35;
//            rotate *= 0.35;
//        }
//
//        if (gamepad2.dpadUpWasPressed()) {flickerUp = true;}
//        if (gamepad2.dpadDownWasPressed()) {flickerUp = false;}
//
//        if (gamepad2.bWasPressed()) {currentIndex = 0; flickerUp = false;}
//        if (gamepad2.yWasPressed()) {currentIndex = 1; flickerUp = false;}
//        if (gamepad2.xWasPressed()) {currentIndex = 2; flickerUp = false;}
//        if (gamepad2.rightTriggerWasPressed()) {shooterActive = true;}
//        if (gamepad2.leftTriggerWasPressed()) {shooterActive = false;}
//
//        if (shooterActive) {
//            shootMotor.setVelocity(launchSpeed);
//        } else {
//            shootMotor.setPower(0);
//        }

//        if (flickerUp) {flickerServo.setPosition(FLICKER_UP_POSE);}
//        else { flickerServo.setPosition(FLICKER_DOWN_POSE);}
//        indexServo.setPosition(TURNTABLE_POSITIONS[currentIndex]);
//        intake.setPower(gamepad2.left_stick_y);

        ManualDrive.driveOrHold(follower, forward, strafe, rotate);


//        telemetry.addData("Runtime", getRuntime());
//        telemetry.addData("Flicker up", flickerUp);
//        telemetry.addData("Current turntable index", currentIndex);
//        telemetry.addData("Launch Speed", launchSpeed);
//        telemetry.addData("Slowmode active", slowmode);
//        telemetry.addData("Shooter active", shooterActive);
//        telemetry.update();
        follower.update();
    }
}
