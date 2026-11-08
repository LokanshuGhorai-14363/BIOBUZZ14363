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

        return null;
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftRear");
        c.backRightName.set("rightRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(7.340251892570436);
        c.yPodOffset.set(2.6424675288162836);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.32914174966101917);
                Controller secondaryTranslationalForward = Controller.proportional(0.12160908167021896);
                Controller primaryTranslationalLateral = Controller.proportional(0.609041204479226);
                Controller secondaryTranslationalLateral = Controller.proportional(0.22502445117437003);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.015421675949597405));
                c.brake.set(Controller.proportionalFeedforward(0.005108424557157794));

                c.headingFeedback.set(Controller.proportional(5.801001776216481));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04587718920695873, 0.005628477656705229));

                c.linearBrakeCoefficients.set(Matrix.diag(0.025344979934771425, 0.0415389397662847));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00217969080797137366, 0.0016284290210533882));

                c.maxAchievableForwardVelocity.set(72.06773287067037);
                c.maxAchievableStrafeVelocity.set(58.831436550264414);
                c.naturalForwardDeceleration.set(70.5197244660409);
                c.naturalStrafeDeceleration.set(97.05050907487711);
            }
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return null;
    }
}