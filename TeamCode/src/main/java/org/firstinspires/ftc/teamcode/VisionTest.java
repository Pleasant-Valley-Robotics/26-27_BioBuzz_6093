package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name="Track balls")
public class VisionTest extends OpMode {
    private Follower follower;
    private Vision vision;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        vision = new Vision(hardwareMap);
        vision.setPipeline(0);
    }

    @Override
    public void loop() {
        double forward = 0;
        double strafe = 0;
        double rotate = 0;

        int index = 0;
        if (gamepad1.aWasPressed()) {
            index++;
            vision.setPipeline(index % 2);
        }

        double targetX = vision.update().getTx();

        rotate = vision.turnTo(0, 0.5);


        ManualDrive.driveOrHold(follower,
                forward,
                strafe,
                rotate);

        follower.update();

        telemetry.addData("Target x", targetX);
        telemetry.addData("Rotate", rotate);
        telemetry.addData("Pipeline", vision.getPipeline());
        telemetry.addData("Frames", vision.getFrames());
        telemetry.update();
    }
}
