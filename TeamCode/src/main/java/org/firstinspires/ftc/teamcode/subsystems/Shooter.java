package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotorEx shootMotor;
    private CRServo windmillServo = null;

    public Shooter(HardwareMap h) {
        shootMotor = h.get(DcMotorEx.class, "shootMotor");
        windmillServo = h.get(CRServo.class, "windmillServo");

        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    pub

}
