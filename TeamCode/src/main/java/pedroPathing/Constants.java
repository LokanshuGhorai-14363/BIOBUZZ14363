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
 * Pedro 3 config for the drivetrain. Pod offsets/directions and Foresight gains
 * below are placeholders and must be replaced with the Quickstart auto-tuner output
 * (Pinpoint -> Mecanum -> Foresight) before the robot drives straight (needs robot test).
 */
public class Constants {

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(NamingConstants.Pinpoint.pinpoint);
        // TODO (needs robot test): paste podType, xPodOffset, yPodOffset,
        // xPodDirection, yPodDirection from the Pinpoint auto-tuner.
    });

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(NamingConstants.Drivetrain.frontLeftMotor);
        c.frontRightName.set(NamingConstants.Drivetrain.frontRightMotor);
        c.backLeftName.set(NamingConstants.Drivetrain.backLeftMotor);
        c.backRightName.set(NamingConstants.Drivetrain.backRightMotor);
        // TODO (needs robot test): flip any direction that drives backwards or
        // strafes the wrong way once the robot is on the ground.
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // TODO (needs robot test): paste controller gains, brake coefficients,
        // and max velocities from the Foresight auto-tuner.
    });

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
