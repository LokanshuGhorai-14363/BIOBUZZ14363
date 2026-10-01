package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

/**
 * DriveSubsystem — Mecanum drive powered by Pedro Pathing's Follower.
 *
 * In TeleOp the Follower handles all kinematic math (inverse kinematics,
 * heading correction, etc.).  We simply feed it translation/strafe/rotation
 * vectors every loop and call {@code update()}.
 *
 * Motor configuration (goBILDA Yellow Jacket 425 RPM) is handled through
 * the Pedro Pathing constants files {@link FConstants} and {@link LConstants}.
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

    // ───────────────────────────── Path following ─────────────────────────────

    /**
     * Begin autonomously following a path to the given target pose.
     * Builds a BezierLine from the current pose to the target.
     *
     * @param target the destination pose (x, y, heading in radians)
     */
    public void followPathTo(Pose target) {
        Pose current = follower.getPose();
        Path path = new Path(new BezierLine(
                new Point(current.getX(), current.getY(), Point.CARTESIAN),
                new Point(target.getX(), target.getY(), Point.CARTESIAN)
        ));
        path.setLinearHeadingInterpolation(current.getHeading(), target.getHeading());

        follower.followPath(path, true);
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
        follower.breakFollowing();
        isFollowingPath = false;
        follower.startTeleopDrive();
    }

    /**
     * Resume TeleOp driving mode after autonomous path completion.
     */
    public void resumeTeleOp() {
        isFollowingPath = false;
        follower.startTeleopDrive();
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
        return follower.getPose();
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
        telemetry.addData("Path Following", isFollowingPath);
    }

    /**
     * Provides the Follower instance for autonomous path-following.
     */
    public Follower getFollower() {
        return follower;
    }
}
