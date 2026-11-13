package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FLDrive");
        c.frontRightName.set("FRDrive");
        c.backLeftName.set("BLDrive");
        c.backRightName.set("BRDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-3.8926979125015384);
        c.yPodOffset.set(-2.003398204412986);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2274613754525742);
                Controller secondaryTranslationalForward = Controller.proportional(0.08404090034983615);
                Controller primaryTranslationalLateral = Controller.proportional(0.21285053451180705);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0786425850310829);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.026803116675862874));
                c.brake.set(Controller.proportionalFeedforward(0.022782649174483444));

                c.headingFeedback.set(Controller.proportional(3.850092690982826));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04386516671871806, 0.004175960358260636));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06291972116757726, 0.05902119627399008));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013035034649855526, 0.0011872776357848588));

                c.maxAchievableForwardVelocity.set(39.8360888987279);
                c.maxAchievableStrafeVelocity.set(32.34054778391909);
                c.naturalForwardDeceleration.set(35.77207736111861);
                c.naturalStrafeDeceleration.set(59.531119555154895);
            }
    );

}