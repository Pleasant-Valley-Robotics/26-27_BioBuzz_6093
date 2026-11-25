package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class Intake {
    // This class is pretty simple, not much of anything beyond basic setPower commands
    private final DcMotorEx intakeMotor;
    private final CRServo leftIntakeServo;
    private final CRServo rightIntakeServo;

    public Intake(HardwareMap h) {
        intakeMotor = h.get(DcMotorEx.class, "intakeMotor");
        leftIntakeServo = h.get(CRServo.class, "leftIntakeServo");
        rightIntakeServo = h.get(CRServo.class, "rightIntakeServo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Make sure we set our motor directions
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);

        // This is set as RUN_USING_ENCODER despite the fact that we don't have an encoder plugged into the intake motor
        // This is pretty stupid, but it doesn't actually matter cause we only use setPower commands, not setVelocity commands
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

    // Pedro pathing takes these things called commands to run autos.
    // They're pretty annoying, especially because you end up with like 10 pairs of parenthesis at once and the IDE doesn't help at all
    // However, you can't do anything other than drive without them in auto, so they are necessary
    public Command setPowerC(double power) { // Returns an instant(). An instant command runs once normal function instantly.
        return instant(() -> // Doing this lambda converts our java function into a pedro pathing command
                setPower(power)
        );
    }

    public Command stopC() {
        // When you don't need any parameters, you can do this::function.
        return instant(this::stop);
    }
    public double getPower() {return intakeMotor.getPower();}


}
