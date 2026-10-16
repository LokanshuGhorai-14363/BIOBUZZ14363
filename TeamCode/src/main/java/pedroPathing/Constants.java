package pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Constants.NamingConstants;

/**
 * Pedro 3 config for the drivetrain.
 *
 * WARNING - this config is incomplete and createFollower() currently THROWS.
 * PinpointConfig and ForesightConfig declare most of their fields as
 * ConfigVar.required(), and reading an unset required var throws
 * IllegalStateException("Config variable has not been set"). Until the TODO blocks
 * below are filled in, every OpMode that builds a Follower (Red/Blue TeleOp) crashes
 * during init. This is not a "drives a bit crooked" placeholder.
 *
 * All the missing values come from the Pedro AutoTune procedures registered in
 * Tuning.java. Deploy this app to the Robot Controller, connect a laptop/phone to the
 * robot's network, and open http://192.168.43.1:10158 - AutoTune runs the procedures
 * from that browser UI and prints a ready-to-paste Java block for each one. There is
 * no tuning OpMode to pick on the Driver Station. Run them in order: Pinpoint Tuner,
 * then Mecanum Tuner, then Foresight Tuner (Tuning.java hands Foresight a localizer and
 * drivetrain built from the two configs below, so it cannot run until they are correct).
 */
public class Constants {

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(NamingConstants.Pinpoint.pinpoint);
        // TODO (needs robot test): replace this whole lambda body with the block printed
        // by the "Pinpoint Tuner" procedure in the AutoTune UI. It walks you through
        // pushing the robot forward, pushing it left, and spinning it 180 degrees, then
        // emits exactly these fields:
        //     c.name.set("pinpoint");
        //     c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //     c.xPodOffset.set(...);     // inches, forward pod offset from center of rotation
        //     c.yPodOffset.set(...);     // inches, strafe pod offset from center of rotation
        //     c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //     c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //     c.globalDistanceUnit.set(DistanceUnit.INCH);
        //     c.offsetUnits.set(DistanceUnit.INCH);
        // Keep the name sourced from NamingConstants rather than the literal the tuner
        // prints, so the device name stays in one place.
        // Required (will throw if left unset): xPodDirection, yPodDirection,
        // xPodOffset, yPodOffset.
    });

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(NamingConstants.Drivetrain.frontLeftMotor);
        c.frontRightName.set(NamingConstants.Drivetrain.frontRightMotor);
        c.backLeftName.set(NamingConstants.Drivetrain.backLeftMotor);
        c.backRightName.set(NamingConstants.Drivetrain.backRightMotor);
        // TODO (needs robot test): confirm these four directions with the "Mecanum Tuner"
        // procedure in the AutoTune UI. With the robot on blocks it spins one motor at a
        // time and asks whether that wheel turned forward or reversed, then emits the
        // matching DcMotorSimple.Direction for each. Unlike the two configs around it,
        // these are real values rather than required-but-unset, so a wrong guess here
        // drives crooked instead of crashing - verify before trusting them.
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // TODO (needs robot test): replace this whole lambda body with the block printed
        // by the "Foresight Tuner" procedure in the AutoTune UI. Run it last - it builds
        // a localizer and drivetrain from the two configs above, so both must already be
        // filled in and correct. It drives the robot through its own forward/strafe/turn
        // maneuvers (give it clear floor space) and emits exactly these fields:
        //     c.forwardTranslational.set(Controller.piecewise(...).put(2.5, ...));
        //     c.strafeTranslational.set(Controller.piecewise(...).put(2.5, ...));
        //     c.coast.set(Controller.proportionalFeedforward(...));
        //     c.brake.set(Controller.proportionalFeedforward(...));
        //     c.headingFeedback.set(Controller.proportional(...));
        //     c.headingBrakeCoefficients.set(Vector2D.cartesian(..., ...));
        //     c.linearBrakeCoefficients.set(Matrix.diag(..., ...));
        //     c.quadraticBrakeCoefficients.set(Matrix.diag(..., ...));
        //     c.maxAchievableForwardVelocity.set(...);   // inches/sec
        //     c.maxAchievableStrafeVelocity.set(...);    // inches/sec
        //     c.naturalForwardDeceleration.set(...);     // inches/sec^2, motors floating
        //     c.naturalStrafeDeceleration.set(...);      // inches/sec^2, motors floating
        // Those twelve are exactly the required() fields on ForesightConfig, so paste the
        // whole block - a partial paste still throws. Everything else on ForesightConfig
        // (maxPathSpeed, brakeAtEnd, the deviation tolerances, ...) has a default and is
        // only worth touching once the robot follows paths well.
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
