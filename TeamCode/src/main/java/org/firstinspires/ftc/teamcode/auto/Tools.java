package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;
import static com.pedropathing.ivy.groups.Groups.sequential;
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

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


public class Tools {
    public Command sleep(long ms) {
        return waitMs(ms);
    }

    public Command shootOnce(double power, Shooter s) {
        return sequential()
    }

    public Command shootAll(double power, Shooter s) {

    }

    public Command intakeOn(Intake i) {

    }

    public Command intakeOff(Intake i) {

    }
}
