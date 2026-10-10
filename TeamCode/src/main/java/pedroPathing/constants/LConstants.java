package pedroPathing.constants;

import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.PinpointConstants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * LConstants — Pedro Pathing localizer configuration for GoBilda Pinpoint.
 * Configure this file for your GoBilda Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry).
 * The values below are starting-point estimates and MUST be tuned on-robot.
 */
public class LConstants {

    static {
        // ── Hardware map name of the Pinpoint device ──────────────────────
        PinpointConstants.hardwareMapName = "pinpoint";

        // ── Distance unit for offsets ─────────────────────────────────────
        PinpointConstants.distanceUnit = DistanceUnit.MM;

        // ── Pod offsets from center of rotation ───────────────────────────
        // forwardY: Y pod offset (forward/backward distance from tracking center)
        // strafeX: X pod offset (left/right distance from tracking center)
        PinpointConstants.forwardY = -168.0;  // tune on-robot
        PinpointConstants.strafeX  = -84.0;   // tune on-robot

        // ── Encoder directions ────────────────────────────────────────────
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection  = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        // ── Odometry pod resolution ───────────────────────────────────────
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;

        // ── Yaw scalar ────────────────────────────────────────────────────
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
    }
}
