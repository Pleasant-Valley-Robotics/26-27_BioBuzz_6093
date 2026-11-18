package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.tools.AutoTools;
import org.firstinspires.ftc.teamcode.tools.Storage;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Auto - 16 Shot", group = "Autonomous")
public class Shoot16 extends LinearOpMode {

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Vision vision;

    private Pose start;
    private final Pose shootFar = new Pose(56.911, 125.529);
    private final Pose shootClose = new Pose(60.15, 10.77);
    private final Pose safeZoneClose = new Pose(44.5, 23.87, 90);
    private final Pose safeZoneFar = new Pose(56.05, 107.43, 90);

    private final AutoTools.Obstacle CLOSE_DANGER_ZONE = new AutoTools.Obstacle(
            new ArrayList<>(List.of(new Pose(46.674, 36.187), new Pose(71.004, 32.986),
                    new Pose(71.510, 63.437), new Pose(46.505, 63.942)))
    );

    private final AutoTools.Obstacle FAR_DANGER_ZONE = new AutoTools.Obstacle(
            new ArrayList<>(List.of(new Pose(71.401, 102.180), new Pose(46.902, 99.654),
                    new Pose(47.239, 63.139), new Pose(71.401, 62.6339)))
    );
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose park = poseFactory.of(16, 96, 270);

    private boolean repositioning = false;
    private boolean hasBalls = true;



    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
            deadline(
                    waitMs(26000),
                    //follow(follower, shootPosition),

                    race(
                            waitUntil(() -> !hasBalls),
                            Groups.loop(lazy(this::shootOrReposition))
                    ),

                    //intakeFromTheWall,
                    setHasBalls(true),

                    race(
                            waitUntil(() -> !hasBalls),
                            Groups.loop(lazy(this::shootOrReposition))
                    ),

                    //Go to flower X,
                    lazy(this::intakeFlowerX),
                    setHasBalls(true),

                    race(
                            waitUntil(() -> !hasBalls),
                            Groups.loop(lazy(this::shootOrReposition))
                    ),

                    // Go to flower Y,
                    lazy(this::intakeFlowerY),
                    setHasBalls(true),

                    race(
                            waitUntil(() -> !hasBalls),
                            Groups.loop(lazy(this::shootOrReposition))
                    )

            ),
            conditional(
                    this::inCollisionZone,
                    sequential(
                        lazy(() -> follow(follower, findSafety())),
                        lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), park)))
                    ),
                    lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), park)))
            )
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        vision = new Vision(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

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

    private boolean inCollisionZone() {
        return (CLOSE_DANGER_ZONE.pointInBounds(follower.pose()) || FAR_DANGER_ZONE.pointInBounds(follower.pose()));
    }

    private boolean hiveUp() {
        return !(vision.findErrorForAutoLocking(Storage.color, (follower.pose().y() > 70.75)) == -999999);
    }

    private Path findSafety() {
        if (CLOSE_DANGER_ZONE.pointInBounds(follower.pose())) {
            return AutoTools.buildPathLinearLine(follower.pose(), safeZoneClose);
        } else {
            return AutoTools.buildPathLinearLine(follower.pose(), safeZoneFar);
        }
    }

    private Path goToOtherGoal() {
        return null;
    }

    private Command shootOrReposition() {
        return sequential(
            setRepositioningC(true),
            deadline(
                    waitMs(3000),
                    sequential(
                        waitUntil(
                                this::hiveUp
                        ),
                        shooter.shootAllC(1240),
                        setHasBalls(false),
                        setRepositioningC(false)
                    )
            ),
            conditional(
                    () -> repositioning,
                    sequential(
                            follow(follower, goToOtherGoal())),
                            setRepositioningC(false)
                    ),
                    shooter.stopC()
            );
    }

    private Command setRepositioningC(boolean value) {
        return instant(() -> repositioning = value);
    }

    private Command setHasBalls(boolean value) {
        return instant(() -> hasBalls = value);
    }

    private Command intakeFlowerX() {
        Pose startPose = follower.pose();
        return sequential(
           intake.setPowerC(1),
            repeat(
                    sequential(
                            follow(follower, AutoTools.buildPathLinearLine(startPose, new Pose(startPose.x() + 5.3, startPose.y()))),
                                    lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), startPose)))
                    ),
                    4
            ),
            intake.setPowerC(0)
        );
    }

    private Command intakeFlowerY() {
        Pose startPose = follower.pose();
        return sequential(
                intake.setPowerC(1),
                repeat(
                        sequential(
                                follow(follower, AutoTools.buildPathLinearLine(startPose, new Pose(startPose.x(), startPose.y() + 5.3))),
                                lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), startPose)))
                        ),
                        4
                ),
                intake.setPowerC(0)
        );
    }





}
