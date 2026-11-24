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
        c.frontLeftName.set("left_front_drive");
        c.frontRightName.set("right_front_drive");
        c.backLeftName.set("left_back_drive");
        c.backRightName.set("right_back_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.5682574294683501);
        c.yPodOffset.set(-1.7741454116941437);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2117982780848101);
                Controller secondaryTranslationalForward = Controller.proportional(0.07825380439811708);
                Controller primaryTranslationalLateral = Controller.proportional(0.2654988547517846);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09809473257072858);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017009669047966643));
                c.brake.set(Controller.proportionalFeedforward(0.014458218690771647));

                c.headingFeedback.set(Controller.proportional(3.6947705915218343));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04531408418670904, 0.0059831743061881555));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07735866431078887, 0.06766347123457163));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.661424721184762E-4, 0.0013075361887354589));

                c.maxAchievableForwardVelocity.set(60.716601824258454);
                c.maxAchievableStrafeVelocity.set(52.159697476384714);
                c.naturalForwardDeceleration.set(29.809013521735125);
                c.naturalStrafeDeceleration.set(56.3776746737847);
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