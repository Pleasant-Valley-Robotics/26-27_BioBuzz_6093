package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Auto - 1.5 Tip", group = "Autonomous")
public class A3HalfsTip extends LinearOpMode {

    private Intake intake;
    private Shooter shooter;
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(57.446, 10.0657, 270);
    private final Pose path1 = poseFactory.of(57.446, 26, 270);
    private final Pose point2Start = poseFactory.of(57.446, 26, 270);
    private final Pose point2 = poseFactory.of(27.577, 10.5, 180);
    private final Pose point3 = poseFactory.of(11, 10.5, 180);
    private final Pose point4 = poseFactory.of(34.3642, 106.4815, 90);
    private final Pose point4Segment1Heading = poseFactory.of(34.3642, 106.4815, 180);
    private final Pose point4Segment2Start = poseFactory.of(34.3642, 106.4815, 180);
    private final Pose point4Segment2End = poseFactory.of(34.3642, 106.4815, 90);
    private final Pose point5 = poseFactory.of(57.2197, 120.3128, 90);
    private final Pose point6 = poseFactory.of(12, 120.1262, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                parallel(
                    shooter.shootC(1425, 6),
                    intake.setPowerC(1.0)
                ),
                follow(follower, path2()),
                intake.setPowerC(1.0),
                waitMs(250),
                race(
                    follow(follower, path3()),
                    waitMs(2500)
                ),
                intake.stopC(),
                waitMs(250),
                follow(follower, path4()),
                follow(follower, path5()),
                parallel(
                        shooter.shootC(1425, 6),
                        intake.setPowerC(1.0)
                ),
                follow(follower, path6()),
                intake.stopC()
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
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3).with(
                Constants.foresightConfig.maxPathSpeed.at(0.3));
    }

    public Path path4() {
        return Paths.line(point3, point4).heading(Interpolator.piecewise().until(0.5, Interpolator.constant(point4Segment1Heading)).until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    public Path path5() {
        return Paths.line(point4, point5).constant(point5);
    }

    public Path path6() {return Paths.line(point5, point6).constant(point6);}
}
