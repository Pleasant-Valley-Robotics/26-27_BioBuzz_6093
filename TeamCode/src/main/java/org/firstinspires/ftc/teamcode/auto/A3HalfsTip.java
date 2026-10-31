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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Auto - 1.5 Tip", group = "Autonomous")
public class A3HalfsTip extends LinearOpMode {

    private Follower follower;
    private DcMotorEx intakeMotor = null;

    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(57.446, 10.0657, 270);
    private final Pose path1 = poseFactory.of(57.446, 15, 270);
    private final Pose point2Start = poseFactory.of(57.446, 34, 270);
    private final Pose point2 = poseFactory.of(27.577, 10.5, 180);
    private final Pose point3 = poseFactory.of(12, 10.5, 180);
    private final Pose point4 = poseFactory.of(34.3642, 106.4815, 90);
    private final Pose point4Segment1Heading = poseFactory.of(34.3642, 106.4815, 180);
    private final Pose point4Segment2Start = poseFactory.of(34.3642, 106.4815, 180);
    private final Pose point4Segment2End = poseFactory.of(34.3642, 106.4815, 90);
    private final Pose point5 = poseFactory.of(57.2197, 120.3128, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
        );
    }

    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftIntakeMotor");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeMotor");


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
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).heading(Interpolator.piecewise().until(0.5, Interpolator.constant(point4Segment1Heading)).until(1, Interpolator.linear(point4Segment2Start, point4Segment2End)));
    }

    public Path path5() {
        return Paths.line(point4, point5).constant(point5);
    }
}
