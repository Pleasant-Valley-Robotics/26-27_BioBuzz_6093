package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Auto - Park Only", group = "Autonomous")
public class ParkOnly extends LinearOpMode {

    private Intake intake;
    private Shooter shooter;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private Follower follower;
    private final Pose start = poseFactory.of(13, 133, 90);
    private final Pose shoot = poseFactory.of(57.446, 31, 270);
    private final Pose point2Start = poseFactory.of(57.446, 26, 270);
    private final Pose point2 = poseFactory.of(13, 123, 90);
    private final Pose point3 = poseFactory.of(11, 10.5, 180);
    private final Pose point4 = poseFactory.of(34.3642, 96, 270);
    private final Pose point5 = poseFactory.of(34.3642, 31, 270);

    private final Pose point4Segment1Heading = poseFactory.of(34.3642, 106.4815, 180);
    private final Pose point4Segment2Start = poseFactory.of(34.3642, 106.4815, 180);
    private final Pose point4Segment2End = poseFactory.of(34.3642, 106.4815, 90);
    private final Pose point6 = poseFactory.of(12, 120.1262, 90);

    private final Pose park = poseFactory.of(16, 96, 270);



    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())


        );
    }

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();


        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }


    public Path path1() {
        return Paths.line(shoot, point2).constant(point2);
    }





}
