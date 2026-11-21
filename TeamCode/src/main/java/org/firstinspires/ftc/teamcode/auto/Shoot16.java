package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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
import org.firstinspires.ftc.teamcode.tools.DriveTools;
import org.firstinspires.ftc.teamcode.tools.MathTools;
import org.firstinspires.ftc.teamcode.tools.Storage;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Auto - 16 Shot", group = "Autonomous")
public class Shoot16 extends LinearOpMode {

    private Follower follower;
    private Intake intake;
    private Shooter shooter;
    private Vision vision;
    private Pose currentTarget;
    private final Pose start = (Storage.close) ?  new Pose(62.077, 7.989, Math.toRadians(270)): new Pose(64.960, 130.321, Math.toRadians(90)); // UPDATE THIS
    private final Pose shootFar = new Pose(61, 125.529, Math.toRadians(90));
    private final Pose shootClose = new Pose(61, 12.276, Math.toRadians(270));
    private final Pose transitionPose = new Pose(64, 68.948, Math.toRadians(0));
    private final Pose safeZoneClose1 = new Pose(63.5, 18, Math.toRadians(90));
    private final Pose safeZoneClose2 = new Pose(30, 18, Math.toRadians(90));
    private final Pose safeZoneFar = new Pose(63.5, 102.18, Math.toRadians(90));
    private final Pose cornerFar = new Pose(36.359, 110.289, Math.toRadians(90));
    private final Pose cornerClose = new Pose(34.334, 29.3689, Math.toRadians(270));
    private final Pose flowerHorizontal = new Pose(56.8, 128.2, Math.toRadians(100));
    private final Pose gardenStart = new Pose(22.724, 11.704, Math.toRadians(230)); // UPDATE THIS
    private final Pose gardenEnd = new Pose(10.917, 6.642, Math.toRadians(180)); // UPDATE THIS
    private final Pose flowerAlliance = new Pose(14.213, 51.4251, Math.toRadians(180)); // UPDATE THIS
    private Pose park; // UPDATE THIS

    private final AutoTools.Obstacle CLOSE_DANGER_ZONE = new AutoTools.Obstacle(
            new ArrayList<>(List.of(new Pose(46.674, 36.187), new Pose(71.004, 32.986),
                    new Pose(71.510, 63.437), new Pose(46.505, 63.942)))
    );

    private final AutoTools.Obstacle FAR_DANGER_ZONE = new AutoTools.Obstacle(
            new ArrayList<>(List.of(new Pose(71.401, 102.180), new Pose(46.902, 99.654),
                    new Pose(47.239, 63.139), new Pose(71.401, 62.6339)))
    );

    private boolean repositioning = false;
    private boolean hasBalls = true;



    // Autonomous routine
    public Command autoRoutine() {
        return parallel(
            sequential(
                deadline(
                        waitMs(24500), // 26 Seconds to finish otherwise park runs early
                        sequential(
                            shooter.setVelocityC(1255),
                            conditional(
                                    () -> Storage.close, // Drive to position based off start
                                    instant(() -> currentTarget = shootClose),
                                    instant(() -> currentTarget = shootFar)
                            ),

                            waitMs(Storage.MSWaitForTip),
                            lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), currentTarget))),
                            waitMs(20),
                            intake.setPowerC(1.0),

                            race(
                                    waitUntil(() -> !hasBalls), // Wait until we fire all our balls
                                    Groups.loop(lazy(this::shootOrReposition))
                            ),
                            instant(this::swapTarget),

                            //intakeFromTheWall,
                            lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), gardenStart))),
                            intake.setPowerC(1),
                            // Slow down so balls don't go flying
                            deadline(
                                    waitMs(3000),
                                    lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), gardenEnd).with(Constants.foresightConfig.maxPathSpeed.at(0.3))))
                                ),
                            intake.setPowerC(0),
                            setHasBalls(true),


                            preShoot(1255),
                            race(
                                    waitUntil(() -> !hasBalls),
                                    Groups.loop(lazy(this::shootOrReposition))
                            ),


                            // Go to flower X
                            lazy(this::intakeFlowerX),
                            setHasBalls(true),

                            shooter.setVelocityC(1255),
                            lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), currentTarget))),
                            intake.setPowerC(1.0),

                            race(
                                    waitUntil(() -> !hasBalls),
                                    Groups.loop(lazy(this::shootOrReposition))
                            ),

                            //Go to flower Y,
                            lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), flowerAlliance))),
                            lazy(this::intakeFlowerY),
                            setHasBalls(true),

                            preShoot(1255),

                            race(
                                    waitUntil(() -> !hasBalls),
                                    Groups.loop(lazy(this::shootOrReposition))
                            )
                        )
                ),

                shooter.stopC(),
                intake.stopC(),
                lazy(this::goPark)
            ),
            parallel(
                Groups.loop(shooter.updateLightC(!repositioning, false)), // Constantly update light unless blinking logic is happening
                // Dynamically update servo position
                Groups.loop(lazy(() -> instant(() -> vision.autoFindServoPosition(MathTools.calculateDistance(follower.pose(), DriveTools.chooseTarget(follower))))))
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

        if (Storage.parkClose) {
            park =  new Pose(14.474, 86.953, Math.toRadians(230));
        } else {
            park = new Pose(19.376, 123.135, Math.toRadians(121.742));
        }

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

    private Command findSafety() {
        if (CLOSE_DANGER_ZONE.pointInBounds(follower.pose())) {
            return
                sequential(
                    lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), safeZoneClose1))),
                    lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), safeZoneClose2)))
                );
        } else if (FAR_DANGER_ZONE.pointInBounds(follower.pose())) {
            return lazy (() ->follow(follower, AutoTools.buildPathLinearLine(follower.pose(), safeZoneFar)));
        } else {
            return lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), park)));
        }
    }

    private Command goPark() {
        return sequential(
                waitMs(20),
                findSafety(),
                lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), park)))
        );
    }



    private Command shootOrReposition() {
        return sequential(
            setRepositioningC(true),
            deadline(
                    waitMs(500), // 3 Seconds to shoot all
                    sequential(
                        waitUntil(
                                this::hiveUp // Wait until limelight sees april tags
                        ),
                        setRepositioningC(false)
                    )
            ),
            conditional(
                () -> repositioning,
                parallel(
                        blink(),
                        sequential(
                            instant(this::swapTarget),
                            lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), transitionPose))),
                            lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), currentTarget))),
                            setRepositioningC(false)
                        )
                    ),

                sequential(
                    shooter.shootAllC(1255, 0),
                    shooter.stopC(),
                    intake.stopC(),
                    setHasBalls(false)
                )
            )
        );
    }

    private Command setRepositioningC(boolean value) {
        return instant(() -> repositioning = value);
    }

    private Command setHasBalls(boolean value) {
        return instant(() -> hasBalls = value);
    }

    private Command intakeFlowerX() {
        return sequential(
           intake.setPowerC(1),
                    deadline(
                        waitMs(2500),
                        lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), flowerHorizontal))
                    ),
                    deadline(
                        waitMs(50),
                        lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), new Pose(flowerHorizontal.x() + 2, flowerHorizontal.y(), flowerHorizontal.heading()))))
                    )
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
                                follow(follower, AutoTools.buildPathLinearLine(startPose, new Pose(startPose.x(), startPose.y() + 5.3, startPose.heading()))),
                                lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), startPose)))
                        ),
                        4
                ),
                intake.setPowerC(0)
        );
    }

    private Command blink() {
        return Groups.repeat(
                sequential(
                        shooter.setLightC(.3),
                        waitMs(200),
                        shooter.setLightC(0),
                        waitMs(200)
                ), 3
        );
    }

    private void swapTarget() {
        if (currentTarget == shootClose) {
            currentTarget = shootFar;
        } else {
            currentTarget = shootClose;
        }
    }

    private Command preShoot(int velocity) {
        if (currentTarget == shootClose) {
            return sequential(
                    lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), cornerClose))),
                    shooter.setVelocityC(velocity),
                    lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), shootClose))),
                    intake.setPowerC(1.0)
            );
        }
        else {
            return sequential(
                    lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), cornerFar))),
                    shooter.setVelocityC(velocity),
                    lazy(() -> follow(follower, AutoTools.buildPathLinearLine(follower.pose(), shootFar))),
                    intake.setPowerC(1.0)
            );
        }
    }



    private Command error(String error) {
        throw new RuntimeException(error);
    }



}
