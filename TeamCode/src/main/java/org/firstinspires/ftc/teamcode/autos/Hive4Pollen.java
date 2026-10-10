package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

// sets up motors and servos for auto
@Autonomous(name="KitBot Auto 1 Tip")
public class Hive4Pollen extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLDrive = null;
    private DcMotor BLDrive = null;
    private DcMotor FRDrive = null;
    private DcMotor BRDrive = null;
    private DcMotor shooterMotor = null;
    private DcMotor intakeMotor = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;
    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.09448818898;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.7;

    // sets up everything and does actions in game
    public void runOpMode() {
        // This code will run once when you press init
        // (set your subsystems and motors here)
        FLDrive = hardwareMap.get(DcMotor.class, "FLDrive");
        BLDrive = hardwareMap.get(DcMotor.class, "BLDrive");
        FRDrive = hardwareMap.get(DcMotor.class, "FRDrive");
        BRDrive = hardwareMap.get(DcMotor.class, "BRDrive");
        shooterMotor = hardwareMap.get(DcMotor.class, "shootMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Starting at",  "%7d : %7d : %7d : %7d",
                     FLDrive.getCurrentPosition(),
                     BLDrive.getCurrentPosition(),
                     FRDrive.getCurrentPosition(),
                     BRDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();

        shooterMotor.setPower(1.0);
        windmillServo.setPower(1.0);
        sleep(10000);
        shooterMotor.setPower(0.0);
        windmillServo.setPower(0.0);

        encoderDrive(DRIVE_SPEED, 5, 5, 3.0); // Forward 5 inches

        telemetry.addData("Shoot1", "Complete");
        telemetry.update();
        sleep(1000);
    }

    // math for auto
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeouts) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = BLDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = BRDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            BLDrive.setTargetPosition(newLeftTarget);
            FLDrive.setTargetPosition(newLeftTarget);
            BRDrive.setTargetPosition(newRightTarget);
            FRDrive.setTargetPosition(newRightTarget);

            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            BLDrive.setPower(Math.abs(speed));
            BRDrive.setPower(Math.abs(speed));
            FLDrive.setPower(Math.abs(speed));
            FRDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeouts) &&
                    (FLDrive.isBusy() && FRDrive.isBusy())) {

                telemetry.addData("Currently at",  "%7d : %7d : %7d : %7d",
                        FLDrive.getCurrentPosition(),
                        BLDrive.getCurrentPosition(),
                        FRDrive.getCurrentPosition(),
                        BRDrive.getCurrentPosition());
                telemetry.update();
            }

            BRDrive.setPower(0.0);
            FRDrive.setPower(0.0);
            FLDrive.setPower(0.0);
            BLDrive.setPower(0.0);

            BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}