package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Motor Set Speed")
public class MotorSpeedSet extends OpMode {
    private DcMotorEx testMotor;
    private double motorSpeedTPS = 1000;

    public void init() {
        testMotor = hardwareMap.get(DcMotorEx.class, "motor");
        testMotor.setPower(0);
    }

    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            motorSpeedTPS += 10;
        }
        if (gamepad1.dpadDownWasPressed()) {
            motorSpeedTPS -= 10;
        }

        if (gamepad1.right_trigger > 0.5) {
            testMotor.setVelocity(motorSpeedTPS);
        } else {
            testMotor.setPower(0);
        }

        telemetry.addData("Motor Speed target (tps)", motorSpeedTPS);
        telemetry.addData("Motor Speed actual (tps)", testMotor.getVelocity());
        telemetry.addData("Motor Speed target (rpm)", tpsToRPM(motorSpeedTPS));
        telemetry.addData("Motor Speed actual (rpm)", tpsToRPM(testMotor.getVelocity()));

        telemetry.update();
    }

    private double tpsToRPM(double tps) {
        return (tps / 28) * 60;
    }
}
