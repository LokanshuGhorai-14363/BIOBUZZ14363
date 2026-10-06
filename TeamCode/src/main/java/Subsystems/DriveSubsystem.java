package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.constants.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

/**
 * DriveSubsystem — Mecanum drive powered by Pedro Pathing's Follower.
 *
 * In TeleOp the Follower handles all kinematic math (inverse kinematics,
 * heading correction, etc.).  We simply feed it translation/strafe/rotation
 * vectors every loop and call {@code update()}.
 *
 * Configuration is handled through {@link Constants}.
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

        // Initialize Follower using unified Constants factory method
        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
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
        follower.manual(forward, strafe, rotation);
    }

    /**
     * Stop all drivetrain motion.
     */
    public void stop() {
        follower.stop();
    }

    // ───────────────────────────── Lifecycle ───────────────────────────────────

    /**
     * Called every CommandScheduler loop.  Updates the Follower's internal
     * odometry and motor outputs.
     */
    @Override
    public void periodic() {
        follower.update();

        Pose pose = follower.pose();
        telemetry.addData("Drive X", pose.x());
        telemetry.addData("Drive Y", pose.y());
        telemetry.addData("Drive Heading (°)", Math.toDegrees(pose.heading()));
    }

    /**
     * Provides the Follower instance for autonomous path-following.
     */
    public Follower getFollower() {
        return follower;
    }
}
