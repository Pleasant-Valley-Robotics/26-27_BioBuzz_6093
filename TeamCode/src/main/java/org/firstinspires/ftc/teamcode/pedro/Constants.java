package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }


    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftDrive");
        c.frontRightName.set("frontRightDrive");
        c.backLeftName.set("backLeftDrive");
        c.backRightName.set("backRightDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.716166819174458);
        c.yPodOffset.set(-1.6640546002725918);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2524519092158778);
                Controller secondaryTranslationalForward = Controller.proportional(0.09327423481601638);
                Controller primaryTranslationalLateral = Controller.proportional(0.3792635316275866);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1401277408281856);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014361824634002898));
                c.brake.set(Controller.proportionalFeedforward(0.012207550938902462));

                c.headingFeedback.set(Controller.proportional(5.320459230630622));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0391748357995371, 0.01080171406938878));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05723958079434827, 0.06759111781046662));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025370415717983082, 0.0016712307948413038));

                c.maxAchievableForwardVelocity.set(63.41335303916675);
                c.maxAchievableStrafeVelocity.set(48.51100999784953);
                c.naturalForwardDeceleration.set(35.84111391626715);
                c.naturalStrafeDeceleration.set(72.93336195635925);
            }
    );


}