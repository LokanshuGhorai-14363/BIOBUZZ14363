package pedroPathing.constants;

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
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftRear");
        c.backRightName.set("rightRear");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.0);
        c.yPodOffset.set(-0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // End Constraints
                c.parametricTConstraint.set(0.025);
                c.velocityConstraint.set(0.1);
                c.translationalConstraint.set(0.1);
                c.headingConstraint.set(Math.toRadians(0.4));
                c.timeoutConstraint.set(100.0);

                // Foresight Options
                c.headingDriveRatio.set(0.5);
                c.brakeAtEnd.set(true);
                c.translationalDeviationTolerance.set(2.5);
                c.headingDeviationTolerance.set(Math.toRadians(11.25));
                c.cosineScale.set(false);

                // Path Constraints / Braking
                c.brakeAggression.set(1.0);
                c.maxBrakingPower.set(0.2);

                // Motion Constraints
                c.maxAccelerationConstraint.set(ForesightConfig.Constraint.NONE);
                c.maxVelocityConstraint.set(ForesightConfig.Constraint.NONE);
                c.maxDecelerationConstraint.set(ForesightConfig.Constraint.NONE);
                c.coastDownToVelocity.set(0.0);

                // Natural Decelerations (Required by Foresight constructor)
                c.naturalForwardDeceleration.set(85.0);
                c.naturalStrafeDeceleration.set(100.0);

                // Feedback & Feedforward Controllers
                Controller primaryTranslationalForward = Controller.proportional(0.3);
                Controller secondaryTranslationalForward = Controller.proportional(0.1);
                Controller primaryTranslationalLateral = Controller.proportional(0.3);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01));
                c.brake.set(Controller.proportionalFeedforward(0.01));

                // Heading & Braking Coefficients
                c.headingFeedback.set(Controller.proportional(5.0));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05, 0.006));
                c.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.08));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001, 0.001));

                // Hold Point Scaling
                c.holdPointTranslationalScaling.set(1.0);
                c.holdPointHeadingScaling.set(1.0);
            }
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
