package pedroPathing.constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Constants — Unified Pedro Pathing v3.0 configuration.
 *
 * <p>Consolidates drivetrain configuration (Mecanum), localizer configuration
 * (GoBilda Pinpoint), and algorithm configuration (Foresight) into a single
 * setup file following official Pedro Pathing v3 architecture.</p>
 *
 * <h3>Usage</h3>
 * <pre>{@code
 *   Follower follower = Constants.create(hardwareMap);
 * }</pre>
 */
public class Constants {

    // ═══════════════════════════════════════════════════════════════════════════
    //  DRIVETRAIN CONFIGURATION — Mecanum (goBILDA 425 RPM)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Mecanum drivetrain configuration.
     * Hardware map names and motor directions for the four drive motors.
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("leftFront");
                c.backLeftName.set("leftRear");
                c.frontRightName.set("rightFront");
                c.backRightName.set("rightRear");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    // ═══════════════════════════════════════════════════════════════════════════
    //  LOCALIZER CONFIGURATION — GoBilda Pinpoint
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * GoBilda Pinpoint odometry computer configuration.
     * Pod offsets are measured in millimeters from the center of rotation.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.offsetUnits.set(DistanceUnit.MM);

                c.yPodOffset.set(-168.0);
                c.xPodOffset.set(-84.0);

                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
    );

    // ═══════════════════════════════════════════════════════════════════════════
    //  FORESIGHT ALGORITHM CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Foresight path-following algorithm configuration.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                c.forwardTranslational.set(Controller.proportional(0.1));
                c.strafeTranslational.set(Controller.proportional(0.1));
                c.headingFeedback.set(Controller.proportional(2.0));

                c.maxAchievableForwardVelocity.set(50.0);
                c.maxAchievableStrafeVelocity.set(40.0);

                c.naturalForwardDeceleration.set(20.0);
                c.naturalStrafeDeceleration.set(15.0);

                c.coast.set(Controller.proportionalFeedforward(0.005));
                c.brake.set(Controller.proportionalFeedforward(0.005));

                c.linearBrakeCoefficients.set(Matrix.diag(0.5, 0.5));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.01, 0.01));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.5, 0.01));
            }
    );

    // ═══════════════════════════════════════════════════════════════════════════
    //  FOLLOWER FACTORY METHOD
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Creates a fully configured {@link Follower} instance.
     *
     * @param hardwareMap the robot's {@link HardwareMap}
     * @return a ready-to-use {@link Follower}
     */
    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
