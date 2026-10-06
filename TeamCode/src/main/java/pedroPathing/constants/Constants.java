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

/**
 * Unified Pedro Pathing v3 configuration (drivetrain + Pinpoint localizer + Foresight).
 *
 * <h3>Motor Specs</h3>
 * goBILDA Yellow Jacket 5203-2402-0019 (19.2:1 — 425 RPM @ 12 V)
 * <ul>
 *   <li>Encoder TPR: 537.7 ticks/revolution</li>
 *   <li>Free speed: 425 RPM</li>
 *   <li>Stall torque: 1.83 kg·cm (× gear ratio)</li>
 * </ul>
 *
 * Numerical values below are carried over from the previous FConstants / LConstants
 * files. Foresight kinematics that did not exist in those files (max velocities,
 * natural deceleration, and brake coefficient matrices) still need AutoTune.
 */
public class Constants {

    /**
     * Robot mass in kilograms. Not a MecanumConfig field in Pedro v3; preserved
     * from FConstants for documentation and any custom scaling.
     */
    public static final double MASS_KG = 13.6; // ~30 lbs

    /** Preserved from FConstants.zeroPowerAccelerationMultiplier. */
    public static final double ZERO_POWER_ACCELERATION_MULTIPLIER = 4.0;

    /** Preserved from FConstants.centripetalScaling. */
    public static final double CENTRIPETAL_SCALING = 0.0005;

    /** Preserved from FConstants drive PID filter coefficient T. */
    public static final double DRIVE_PID_FILTER_T = 0.6;

    /** Preserved from FConstants drive PID D. */
    public static final double DRIVE_PID_D = 0.0005;

    /**
     * Preserved from LConstants.customEncoderResolution. Not applied because
     * useCustomEncoderResolution was false (stock goBILDA 4-bar pods).
     */
    public static final double CUSTOM_ENCODER_RESOLUTION = 13.26291192;

    /** Preserved from LConstants.yawScalar (unused by PinpointConfig in v3). */
    public static final double YAW_SCALAR = 1.0;

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

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");

                // LConstants used DistanceUnit.MM for pod offsets.
                c.offsetUnits.set(DistanceUnit.MM);
                // strafeX → X pod offset, forwardY → Y pod offset
                c.xPodOffset.set(-84.0);
                c.yPodOffset.set(-168.0);

                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // Translational PID from FConstants (P=0.1, I=0.0, D=0.01, F=0.0)
                Controller translational = Controller.pid(0.1, 0.0, 0.01);
                c.forwardTranslational.set(translational);
                c.strafeTranslational.set(Controller.pid(0.1, 0.0, 0.01));

                // Heading PID from FConstants (P=2.0, I=0.0, D=0.1, F=0.0)
                c.headingFeedback.set(Controller.pid(2.0, 0.0, 0.1));

                // Drive PID P=0.02 maps to Foresight coast/brake kV; drive F=0.0
                c.coast.set(Controller.proportionalFeedforward(0.02));
                c.brake.set(Controller.proportionalFeedforward(0.02));

                // Path-end constraints from FConstants
                // pathEndTValueConstraint = 0.995 → complete when t >= 0.995
                c.parametricTConstraint.set(1.0 - 0.995);
                c.velocityConstraint.set(0.1);
                c.timeoutConstraint.set(500.0);

                // Required Foresight kinematics (not present in F/L constants).
                // Run the Foresight AutoTuner and replace these with generated values.
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
                c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));
                c.maxAchievableForwardVelocity.set(72.72923108818539);
                c.maxAchievableStrafeVelocity.set(52.34323936525474);
                c.naturalForwardDeceleration.set(85.01144677379789);
                c.naturalStrafeDeceleration.set(104.49787535782846);
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
