package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@Configurable
public class Vision {
    private Servo limelightServo;
    private Limelight3A limelight;

    public Vision(HardwareMap h) {
        limelightServo = h.get(Servo.class, "limelightServo");
        limelightServo.setDirection(Servo.Direction.REVERSE);
        limelight = h.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void setServoPosition(double position) {
        limelightServo.setPosition(position);
    }

    public double autoFindServoPosition(double distance) {// Far should be true if we are targeting the far hive
        double kP = .004;
        double start = .59;
        double position = (distance * kP) + start;
        setServoPosition(distance * kP + start);
        return position;
    }

    public double findErrorForAutoLocking(int color, boolean far) { // Far should be true if we are targeting the far hive
        List<LLResultTypes.FiducialResult> aprilTags = limelight.getLatestResult().getFiducialResults();
        double sum = 0;
        int count = 0;

        for (int i = 0; i < aprilTags.size(); i++) {
            if (checkForValidAprilTag(aprilTags.get(i).getFiducialId(), color, far)) {
                sum += aprilTags.get(i).getTargetXDegrees();
                count++;
            }
        }
        /*else if (validAprilTags.size() == 2) {
            if (color == 1) {
                if (far) {
                    if ((validAprilTags.get(0).getFiducialId() == 30 && validAprilTags.get(1).getFiducialId() == 33)
                            || (validAprilTags.get(0).getFiducialId() == 33 && validAprilTags.get(1).getFiducialId() == 30)
                            || (validAprilTags.get(0).getFiducialId() == 32 && validAprilTags.get(1).getFiducialId() == 31)
                            || (validAprilTags.get(0).getFiducialId() == 31 && validAprilTags.get(1).getFiducialId() == 32)) {
                        return (validAprilTags.get(0).getTargetXDegrees() + validAprilTags.get(1).getTargetXDegrees()) / 2;
                    }
                }
            }
        }  else if (validAprilTags.size() == 3){

        } else {  This would be at least 70 lines of code if we were to finish it, but instead we're cheating and doing the easy way*/
        if (count == 2) {
            return sum / count;
        }
        return -999999;
    }

    private boolean checkForValidAprilTag(int aprilTagID, int color, boolean far) {
        if (color == 1) { // THIS IS REVERSED BECAUSE WE RESET BLUE TO RED ON BLUE SIDE
            if (!far) {
                return (aprilTagID == 42 || 45 == aprilTagID);
            } else {
                return (aprilTagID == 38 || 41 == aprilTagID);
            }
        } else {
            if (far) {
                return (aprilTagID == 30 || 33 == aprilTagID);
            } else {
                return (aprilTagID == 34 || 37 == aprilTagID);
            }
        }
    }
}
