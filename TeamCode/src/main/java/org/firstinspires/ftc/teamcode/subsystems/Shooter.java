package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import java.util.ArrayList;

public class Shooter {
    private final DcMotorEx shootMotor;
    private final CRServo windmillServo;

    private Intake intake;

    public Shooter(HardwareMap h) {
        shootMotor = h.get(DcMotorEx.class, "shootMotor");
        windmillServo = h.get(CRServo.class, "windmillServo");

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void windmillSetPower(double power) {
        windmillServo.setPower(power);
    }

    public void setVelocity(int velocity) {
        shootMotor.setVelocity(velocity);
    }

    public void stop() {
        shootMotor.setPower(0);
        windmillServo.setPower(0);
    }

    public Command setVelocityC(int velocity) {
        return instant(() -> setVelocity(velocity));
    }

    public Command windmillSetPowerC(double power) {
        return instant(() -> windmillSetPower(power));
    }

    public Command stopC() {
        return sequential(setVelocityC(0), windmillSetPowerC(0));
    }


    public Command shootC(int velocity, int shots) {
        return sequential(
                setVelocityC(velocity),

                waitMs(2500),
                repeat(
                        sequential(
                                windmillSetPowerC(-0.15),
                                waitMs(1150),
                                windmillSetPowerC(0),
                                waitMs(1300)
                        ), shots
                ),
                stopC()
        );
    }

    public Command shootContinuous(int velocity) {
        return sequential(
                setVelocityC(velocity),
                waitMs(2500),
                windmillSetPowerC(-0.11),
                waitMs(5000),
                windmillSetPowerC(0),
                stopC()
        );
    }

    public double getVelocity() {
        return shootMotor.getVelocity();
    }






}
