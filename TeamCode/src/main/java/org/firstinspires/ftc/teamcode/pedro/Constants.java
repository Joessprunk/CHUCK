package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
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


    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FL_drive_motor");
        c.frontRightName.set("FR_drive_motor");
        c.backLeftName.set("RL_drive_motor");
        c.backRightName.set("RR_drive_motor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(9.582187922920768);
        c.yPodOffset.set(-4.042431989054042);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.36210906483823596);
                Controller secondaryTranslationalForward = Controller.proportional(0.1337896237253152);
                Controller primaryTranslationalLateral = Controller.proportional(0.5229515073133536);
                Controller secondaryTranslationalLateral = Controller.proportional(0.19321660843065472);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01537760752625679));
                c.brake.set(Controller.proportionalFeedforward(0.01307096639731827));

                c.headingFeedback.set(Controller.proportional(5.870034108134855));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06562537953111745, 0.005734285675190217));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11627933971431828, 0.08044203172616182));
                c.quadraticBrakeCoefficients.set(Matrix.diag(8.745637630502973E-4, 0.0013559575517786407));

                c.maxAchievableForwardVelocity.set(65.32214097589534);
                c.maxAchievableStrafeVelocity.set(49.08272043205203);
                c.naturalForwardDeceleration.set(43.049702972374554);
                c.naturalStrafeDeceleration.set(74.47875053702192);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}