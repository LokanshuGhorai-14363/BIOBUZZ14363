package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

/**
 * LConstants — Pedro Pathing localizer configuration.
 *
 * Configure this file for your specific encoder setup (three dead-wheel
 * odometry).  The values below are placeholders for a typical goBILDA
 * encoder pod setup and MUST be tuned on-robot using the Pedro Pathing
 * tuning OpModes.
 */
public class LConstants {

    static {
        // ── Encoder directions ────────────────────────────────────────────
        // Set these based on which direction each encoder reads positive
        // when the robot moves forward / strafes right.
        ThreeWheelConstants.forwardTicksToInches   = 0.00297;  // tune via ForwardPushTest
        ThreeWheelConstants.strafeTicksToInches    = 0.00297;  // tune via StrafePushTest
        ThreeWheelConstants.turnTicksToInches      = 0.00297;  // tune via TurnTest

        // ── Encoder hardware names ────────────────────────────────────────
        ThreeWheelConstants.leftEncoder            = "leftFront";
        ThreeWheelConstants.rightEncoder           = "rightFront";
        ThreeWheelConstants.strafeEncoder          = "leftRear";

        // ── Encoder directions (FORWARD / REVERSED) ──────────────────────
        ThreeWheelConstants.leftEncoderDirection    = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection   = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection  = Encoder.FORWARD;

        // ── Pod offsets from center of rotation (inches) ──────────────────
        // Positive X = forward, Positive Y = left
        ThreeWheelConstants.leftY                  =  6.5;   // tune on-robot
        ThreeWheelConstants.rightY                 = -6.5;   // tune on-robot
        ThreeWheelConstants.strafeX                = -5.0;   // tune on-robot
    }
}
