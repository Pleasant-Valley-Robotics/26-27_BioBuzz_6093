package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class VisionTest extends OpMode {
    private Follower follower;
    private Vision vision;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        vision = new Vision(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = 0;
        double strafe = 0;
        double rotate = 0;
    }
}
