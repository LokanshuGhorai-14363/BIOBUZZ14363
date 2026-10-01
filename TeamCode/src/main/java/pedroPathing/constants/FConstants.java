package pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * FConstants — Pedro Pathing follower / drive configuration.
 *
 * <h3>Motor Specs</h3>
 * goBILDA Yellow Jacket 5203-2402-0019 (19.2:1 — 425 RPM @ 12 V)
 * <ul>
 *   <li>Encoder TPR: 537.7 ticks/revolution</li>
 *   <li>Free speed: 425 RPM</li>
 *   <li>Stall torque: 1.83 kg·cm (× gear ratio)</li>
 * </ul>
 *
 * All PID / feedforward values below are **starting-point estimates**.
 * They MUST be tuned on-robot using Pedro Pathing's built-in tuning
 * OpModes (TranslationalPIDTuner, HeadingPIDTuner, DrivePIDTuner).
 */
public class FConstants {

    static {
        // ── Localizer selection ───────────────────────────────────────────
        FollowerConstants.localizers = Localizers.PINPOINT;

        // ── Drive motor configuration ─────────────────────────────────────
        // Hardware map names for the four mecanum drive motors
        FollowerConstants.leftFrontMotorName  = "leftFront";
        FollowerConstants.leftRearMotorName   = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName  = "rightRear";

        // goBILDA 425 RPM motor direction — set true if motor is reversed
        FollowerConstants.leftFrontMotorDirection  = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection   = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection  = DcMotorSimple.Direction.FORWARD;

        // ── Mass of the robot (kg) ────────────────────────────────────────
        FollowerConstants.mass = 13.6;  // ~30 lbs — adjust to your robot

        // ── Translational PID ─────────────────────────────────────────────
        // Controls robot position on the field (X/Y)
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(
                0.1,   // P
                0.0,   // I
                0.01,  // D
                0.0    // F
        );

        // ── Heading PID ───────────────────────────────────────────────────
        // Controls robot heading (rotation)
        FollowerConstants.headingPIDFCoefficients.setCoefficients(
                2.0,   // P
                0.0,   // I
                0.1,   // D
                0.0    // F
        );

        // ── Drive PID ─────────────────────────────────────────────────────
        // Controls forward/backward drive power during path following
        FollowerConstants.drivePIDFCoefficients.setCoefficients(
                0.02,  // P
                0.0,   // I
                0.0005,// D
                0.6    // F (feedforward — fraction of max velocity)
        );

        // ── Path following parameters ─────────────────────────────────────
        FollowerConstants.zeroPowerAccelerationMultiplier = 4.0;
        FollowerConstants.centripetalScaling              = 0.0005;

        // ── Path end timeouts ─────────────────────────────────────────────
        FollowerConstants.pathEndTimeoutConstraint   = 500;  // ms
        FollowerConstants.pathEndTValueConstraint     = 0.995;
        FollowerConstants.pathEndVelocityConstraint   = 0.1;
    }
}
