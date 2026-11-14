package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Vision {
    private Servo limelightServo;
    private Limelight3A limelight;
    public Vision(HardwareMap h) {
        limelightServo = h.get(Servo.class, "limelightServo");
        limelight = h.get(Limelight3A.class, "limelight");
    }

    public void setServoPosition(double position) {
        limelightServo.setPosition(position);
    }
}
