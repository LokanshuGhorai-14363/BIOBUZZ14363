package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

/**
 * DriveSubsystem — Mecanum drive powered by Pedro Pathing's Follower.
 *
 * In TeleOp the Follower handles all kinematic math (inverse kinematics,
 * heading correction, etc.).  We simply feed it translation/strafe/rotation
 * vectors every loop and call {@code update()}.
 *
 * Motor configuration (goBILDA Yellow Jacket 425 RPM) is handled through
 * the Pedro Pathing constants files {@link FConstants} and {@link LConstants}.
 */
public class DriveSubsystem extends SubsystemBase {

    private final Follower follower;
    private final Telemetry telemetry;

    // Starting pose — (0, 0, 0) by default; override in autonomous
    private static final Pose START_POSE = new Pose(0, 0, 0);

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     */
    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Follower reads FConstants & LConstants automatically
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(START_POSE);
        follower.startTeleopDrive();
    }

    // ───────────────────────────── TeleOp control ─────────────────────────────

    /**
     * Pass raw joystick values into the Pedro Pathing drive system.
     *
     * @param forward  left-stick Y (negative = forward on most gamepads)
     * @param strafe   left-stick X
     * @param rotation right-stick X (or left-stick X for single-stick mode)
     */
    public void drive(double forward, double strafe, double rotation) {
        follower.setTeleOpMovementVectors(forward, strafe, rotation, true);
    }

    /**
     * Stop all drivetrain motion.
     */
    public void stop() {
        follower.setTeleOpMovementVectors(0, 0, 0, false);
    }

    // ───────────────────────────── Lifecycle ───────────────────────────────────

    /**
     * Called every CommandScheduler loop.  Updates the Follower's internal
     * odometry and motor outputs.
     */
    @Override
    public void periodic() {
        follower.update();

        Pose pose = follower.getPose();
        telemetry.addData("Drive X", pose.getX());
        telemetry.addData("Drive Y", pose.getY());
        telemetry.addData("Drive Heading (°)", Math.toDegrees(pose.getHeading()));
    }

    /**
     * Provides the Follower instance for autonomous path-following.
     */
    public Follower getFollower() {
        return follower;
    }
}
