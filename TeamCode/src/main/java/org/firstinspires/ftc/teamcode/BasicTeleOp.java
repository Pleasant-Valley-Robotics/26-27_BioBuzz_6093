package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="Basic TeleOp")
public class BasicTeleOp extends OpMode {
    private Follower follower;
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx intake;
    private Servo flickerServo;
    private Servo indexServo;

    private boolean shooterActive = false;
    private int launchSpeed = 50000;

    private final double FLICKER_UP_POSE = 0.38;
    private final double FLICKER_DOWN_POSE = 0.43;

    private final double[] TURNTABLE_POSITIONS = {.169, 0.511, .849};
    private int currentIndex = 0;

    private boolean slowmode = false;
    private boolean flickerUp = false;





    public void init() {
        follower = Constants.create(hardwareMap);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flickerServo = hardwareMap.get(Servo.class, "flicker");
        indexServo = hardwareMap.get(Servo.class, "index");

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        slowmode = gamepad1.right_trigger_pressed;

        if (slowmode) {
            forward *= 0.35;
            strafe *= 0.35;
            rotate *= 0.35;
        }

        if (gamepad2.dpadUpWasPressed()) {flickerUp = true;}
        if (gamepad2.dpadDownWasPressed()) {flickerUp = false;}

        if (gamepad2.bWasPressed()) {currentIndex = 0; flickerUp = false;}
        if (gamepad2.yWasPressed()) {currentIndex = 1; flickerUp = false;}
        if (gamepad2.xWasPressed()) {currentIndex = 2; flickerUp = false;}
        if (gamepad2.rightTriggerWasPressed()) {shooterActive = true;}
        if (gamepad2.leftTriggerWasPressed()) {shooterActive = false;}

        if (shooterActive) {
            shooter1.setVelocity(launchSpeed);
            shooter2.setVelocity(launchSpeed);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }

        if (flickerUp) {flickerServo.setPosition(FLICKER_UP_POSE);}
        else { flickerServo.setPosition(FLICKER_DOWN_POSE);}
        indexServo.setPosition(TURNTABLE_POSITIONS[currentIndex]);
        intake.setPower(gamepad2.left_stick_y);

        ManualDrive.driveOrHold(follower, forward, strafe, rotate);


        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Flicker up", flickerUp);
        telemetry.addData("Current turntable index", currentIndex);
        telemetry.addData("Launch Speed", launchSpeed);
        telemetry.addData("Slowmode active", slowmode);
        telemetry.addData("Shooter active", shooterActive);
        telemetry.update();
        follower.update();
    }
}
