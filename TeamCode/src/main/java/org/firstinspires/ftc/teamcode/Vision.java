package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class Vision {
    // This would go in the utils folder
    private Limelight3A limelight;
    private LLResult result;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void setPipeline(int i) {
        limelight.pipelineSwitch(i);
    }



    public LLResult update() {
        result = limelight.getLatestResult();
        return result;
    }


    public double turnTo(double tolerance, double maxSpeed) {
        if (result == null || !result.isValid()) {
            return 0;
        }

        double error = result.getTx();


        if (Math.abs(error) > tolerance) {
            double kP = 0.03;
            double turnPower = -kP * error;

            turnPower = Math.min(maxSpeed, Math.max(turnPower, -maxSpeed));
            return turnPower;
        }

        return 0;
    }


    public String getPipeline() {
        return limelight.getStatus().getPipelineType();
    }

    public int getPipelineID() {
        return limelight.getStatus().getPipelineIndex();
    }

    public double getFrames() {
        return limelight.getStatus().getFps();
    }


}
