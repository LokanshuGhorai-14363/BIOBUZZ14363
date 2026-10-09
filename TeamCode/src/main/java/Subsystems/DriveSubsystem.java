package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.constants.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.AtomicPath;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;

/**
 * DriveSubsystem — Mecanum drive powered by Pedro Pathing's Follower.
 *
 * In TeleOp the Follower handles all kinematic math (inverse kinematics,
 * heading correction, etc.).  We simply feed it translation/strafe/rotation
 * vectors every loop and call {@code update()}.
 *
 * Motor configuration (goBILDA Yellow Jacket 425 RPM) is handled through
 * the Pedro Pathing constants file {@link Constants}.
 *
 * Also supports autonomous path-following for relocalization and pose
 * overrides from vision data.
 */
public class DriveSubsystem extends SubsystemBase {

    private final Follower follower;
    private final Telemetry telemetry;

    /** Set to true while the robot is autonomously following a path. */
    private boolean isFollowingPath = false;

    // Starting pose — (0, 0, 0) by default; override in autonomous
    private static final Pose START_POSE = new Pose(0, 0, 0);

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     */
    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Create Follower using Pedro Pathing v3 Constants factory method
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
        follower.manual(0, 0, 0);
    }

    // ───────────────────────────── Path following ─────────────────────────────

    /**
     * Begin autonomously following a path to the given target pose.
     * Builds a Line path from the current pose to the target.
     *
     * @param target the destination pose (x, y, heading in radians)
     */
    public void followPathTo(Pose target) {
        Pose current = follower.pose();
        Path path = new AtomicPath(new Line(current, target)).linear(current, target);

        follower.follow(path);
        isFollowingPath = true;
    }

    /**
     * @return {@code true} if the Follower has completed its current path.
     */
    public boolean isPathComplete() {
        if (!isFollowingPath) return true;
        boolean done = !follower.isBusy();
        if (done) {
            isFollowingPath = false;
        }
        return done;
    }

    /**
     * @return {@code true} if the robot is currently autonomously following a path.
     */
    public boolean isFollowingPath() {
        return isFollowingPath;
    }

    /**
     * Abort any active path follow and resume TeleOp control.
     */
    public void cancelPath() {
        follower.stop();
        isFollowingPath = false;
    }

    /**
     * Resume TeleOp driving mode after autonomous path completion.
     */
    public void resumeTeleOp() {
        isFollowingPath = false;
    }

    // ───────────────────────────── Pose management ────────────────────────────

    /**
     * Override the Follower's internal pose (e.g., from vision relocalization).
     *
     * @param pose the corrected pose from vision data
     */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /**
     * @return the current estimated robot pose from odometry.
     */
    public Pose getPose() {
        return follower.pose();
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
        telemetry.addData("Path Following", isFollowingPath);
    }

    /**
     * Provides the Follower instance for autonomous path-following.
     */
    public Follower getFollower() {
        return follower;
    }
}
