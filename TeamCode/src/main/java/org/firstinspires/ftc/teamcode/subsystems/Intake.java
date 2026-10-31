package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class Intake {
    private final DcMotorEx intakeMotor;
    private final CRServo leftIntakeServo;
    private final CRServo rightIntakeServo;

    public Intake(HardwareMap h) {
        intakeMotor = h.get(DcMotorEx.class, "intakeMotor");
        leftIntakeServo = h.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = h.get(CRServo.class, "rightIntakeServo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
        leftIntakeServo.setPower(power);
        rightIntakeServo.setPower(power);
    }

    public void stop() {
        setPower(0);
    }

    public Command setPowerC(double power) {
        return instant(() ->
                setPower(power)
        );
    }

    public Command stopC() {
        return instant(this::stop);
    }
    public double getPower() {return intakeMotor.getPower();}


}
