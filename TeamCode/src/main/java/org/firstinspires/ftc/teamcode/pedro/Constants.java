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
                Controller primaryTranslationalForward = Controller.proportional(0.23622505962116533);
                Controller secondaryTranslationalForward = Controller.proportional(0.08727884748017682);
                Controller primaryTranslationalLateral = Controller.proportional(0.2616032637588879);
                Controller secondaryTranslationalLateral = Controller.proportional(0.0966554157909617);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.018300196959737215));
                c.brake.set(Controller.proportionalFeedforward(0.015555167415776632));

                c.headingFeedback.set(Controller.proportional(3.4367346928094875));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.046744296641884536, 0.010044131326389765));

                c.linearBrakeCoefficients.set(Matrix.diag(0.059156762758230313, 0.004231461883430998));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00299192067890445, 4.7580528691995946E-4));

                c.maxAchievableForwardVelocity.set(45.871392706221606);
                c.maxAchievableStrafeVelocity.set(38.05845339214077);
                c.naturalForwardDeceleration.set(29.87481096618597);
                c.naturalStrafeDeceleration.set(54.06704836978433);
            }
    );

}