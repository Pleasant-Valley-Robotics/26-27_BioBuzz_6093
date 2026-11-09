package org.firstinspires.ftc.teamcode.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
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

@Configurable
@TeleOp(name="THIS ONE JESSE --->>>>> Teleop for comp")
public class CompTeleop extends OpMode {
    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private Follower follower;
    private DcMotorEx shootMotor;
    private DcMotorEx intakeMotor;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    private Servo light;
    private IMU imu;

    private static int motorSpeed = 1275;
    private int shooterValue = 0;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double f= 5;
    private ElapsedTime runtime = new ElapsedTime();

    private boolean swapColors = true;

    public void init() {
        runtime.reset();
        follower = Constants.create(hardwareMap);

        shootMotor = hardwareMap.get(DcMotorEx.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        imu = hardwareMap.get(IMU.class, "imu");

        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");
        light = hardwareMap.get(Servo.class, "light");

        shootMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shootMotor.setDirection(DcMotorEx.Direction.FORWARD);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);



    }

    public void loop() {
        shootMotor.setVelocityPIDFCoefficients(p, i, d, f);
        shootMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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
        if (shooterValue==1) {
            shootMotor.setVelocity(motorSpeed);
            if (Math.abs(shootMotor.getVelocity() - motorSpeed) < 80) {
                if (swapColors) {
                    light.setPosition(.3);
                } else {
                    light.setPosition(0);
                }
            } else {
                light.setPosition(0);
            }
        } else if (shooterValue==-1) {
            shootMotor.setVelocity(-600);
            light.setPosition(.7);
        } else {
            shootMotor.setPower(0);
            light.setPosition(0);
        }

        if (gamepad1.left_trigger_pressed) {
            forward *= .35;
            strafe *= .35;
            rotate *= .35;
        }
        ManualDrive.driveOrHold(follower, forward, strafe, rotate);

        telemetry.addData("Runtime", runtime);
        telemetry.addData("Target TPS", motorSpeed);
        telemetry.addData("shooterMotor TPS", shootMotor.getVelocity());
        telemetry.addData("shooterMotor RPM", (shootMotor.getVelocity()/26)*60);
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Light on", swapColors);
        telemetry.addData("Applied PIDF Coefficients", shootMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

        telemetry.update();


        panelsTelemetry.getFtcTelemetry().addData("P Value", p);
        panelsTelemetry.getFtcTelemetry().addData("I Value", i);
        panelsTelemetry.getFtcTelemetry().addData("D Value", d);
        panelsTelemetry.getFtcTelemetry().addData("F Value", f);

        panelsTelemetry.getFtcTelemetry().addData("Target TPS", motorSpeed);
        panelsTelemetry.getFtcTelemetry().addData("Current TPS", shootMotor.getVelocity());
        panelsTelemetry.getFtcTelemetry().addData("shooterMotor RPM", (shootMotor.getVelocity()/26)*60);


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
}
