package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision {
    // This would go in the utils folder
    Limelight3A limelight;
    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public Pose3D findPollenWithNeural() {
        return null;
    }

    public Pose3D findPollenWithCV() {
        return null;
    }

}
