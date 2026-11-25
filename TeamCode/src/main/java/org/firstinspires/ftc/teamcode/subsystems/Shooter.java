package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.field.PanelsField;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import java.util.ArrayList;

public class Shooter {
    private final DcMotorEx shootMotor;
    // CRServo stands for Continuous Rotation Servo.
    // Most of the functions are the same, but the way they are sent to the robot changes
    private final CRServo windmillServo;
    private final Servo light;

    private double lastVelocity = 0;


    public Shooter(HardwareMap h) {
        shootMotor = h.get(DcMotorEx.class, "shootMotor");
        windmillServo = h.get(CRServo.class, "windmillServo");
        light = h.get(Servo.class, "light");


        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        windmillServo.setDirection(DcMotorSimple.Direction.FORWARD);
        // RUN_USING_ENCODER makes the motor send back feedback on how fast it's running
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Using that feedback, the robot will adjust to try to reach it's target velocity using PIDF coefficients

        //PIDF is it's whole big thing, look it up if you wanna know more
        shootMotor.setVelocityPIDFCoefficients(90, 0.13, 0.5, 10.75);
    }

    // This isn't used at all right now, but it was used when finding PIDF to update the motor every loop
    public void setPIDFCoefficients(double p, double i, double d, double f) {
        shootMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }
    public void windmillSetPower(double power) {
        windmillServo.setPower(power);
    }


    public void setVelocity(int velocity) {
        shootMotor.setVelocity(velocity);
        // We save our last velocity so that the light can have a target velocity to determine if we really are at speed or not
        lastVelocity = velocity;
    }

    public void setLight(double power) {
        light.setPosition(power);
    }

    public void updateLight(boolean active, boolean swapColor) {
        if (active) {
            if (lastVelocity < 0) {
                light.setPosition(.7); // This is like purple for backwards
            } else if (Math.abs(getVelocity() - lastVelocity) < 50) { // If our velocity is within 50 tps of our target
                if (swapColor) {
                    light.setPosition(.3); // RED
                } else {
                    // The secondary driver can choose to turn off lights altogether if they feel that other teams are playing defense based on the light
                    light.setPosition(0); // power off
                }
            }
        } else {
            light.setPosition(0);  // power off
        }
    }

    public void stop() {
        shootMotor.setPower(0);
        windmillServo.setPower(0);
    }

    public double getVelocity() {
        return shootMotor.getVelocity();
    }

    // Go to intake.java if you want an explanation on pedro pathing commands
    public Command updateLightC(boolean active, boolean swapColor) {
        return instant(() -> updateLight(active, swapColor));
    }
    public Command setLightC(double power) {
        return instant(() ->setLight(power));
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

    // This function return a sequential command, which is basically a collection of other commands run one after another
    public Command shootC(int velocity, int shots) {
        return sequential(
                setVelocityC(velocity),

                waitMs(2500), // This is a built in pedro command that waits for us
                repeat( // This repeats for an integer amount of times. it takes two commands: commandToRun, and then timesToRun
                        sequential(
                                windmillSetPowerC(-0.15),
                                waitMs(1150),
                                windmillSetPowerC(0),
                                waitMs(1300)
                        ), shots // shots is the number of times to run
                ),
                stopC()
        );
    }

    public Command shootAllC(int velocity, int msToSpeedUp) {
        return sequential(
                setVelocityC(velocity),
                waitMs(msToSpeedUp),
                windmillSetPowerC(.3),
                waitMs(2750),
                windmillSetPowerC(0),
                setVelocityC(0)
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






}
