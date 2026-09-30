package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="KitBot Auto")
public class AutoExample extends LinearOpMode {
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
    static final double COUNTS_PER_MOTOR_REV  = 0;

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

        while (opModeInInit()) {
            // This code runs repeatedly while the opmode is in init waiting for start
        }

        // This code will run once when you press start

        while (opModeIsActive()) {
            // This code will run repeatedly until someone presses stop
        }

    }

}
