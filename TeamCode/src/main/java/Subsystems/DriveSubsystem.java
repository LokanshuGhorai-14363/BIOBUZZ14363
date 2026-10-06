package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import pedroPathing.constants.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.config.Modifier;
import com.pedropathing.paths.PathSegment;

import java.util.ArrayList;
import java.util.List;

/**
 * DriveSubsystem — Mecanum drive powered by Pedro Pathing's Follower.
 *
 * In TeleOp the Follower/Drivetrain handles all kinematic math.
 * We simply feed it translation/strafe/rotation vectors every loop and call {@code update()}.
 *
 * Motor configuration is handled through the Pedro Pathing constants file {@link Constants}.
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

        follower = Constants.create(hardwareMap);
        follower.setPose(START_POSE);
    }

    // ───────────────────────────── TeleOp control ─────────────────────────────

    /**
     * Pass raw joystick values into the Pedro Pathing drive system.
     *
     * @param forward  left-stick Y (negative = forward on most gamepads)
     * @param strafe   left-stick X
     * @param rotation right-stick X
     */
    public void drive(double forward, double strafe, double rotation) {
        follower.drivetrain.drive(new DrivePowers(forward, strafe, rotation), true);
    }

    /**
     * Stop all drivetrain motion.
     */
    public void stop() {
        follower.drivetrain.stop();
    }

    // ───────────────────────────── Path following ─────────────────────────────

    /**
     * Begin autonomously following a path to the given target pose.
     * Builds a BezierCurve from the current pose to the target.
     *
     * @param target the destination pose (x, y, heading in radians)
     */
    public void followPathTo(Pose target) {
        Pose current = follower.pose();
        Path path = new Path(new BezierCurve(
                Vector2D.cartesian(current.x(), current.y()),
                Vector2D.cartesian(target.x(), target.y())
        ), new ArrayList<>()) {
            @Override
            public double heading(double v) {
                return target.heading();
            }
            @Override
            protected boolean hasHeading() {
                return true;
            }
            @Override
            protected List<PathSegment> getSegments(PathSegment.HeadingProvider headingProvider, List<Modifier> list) {
                return new ArrayList<>();
            }
            @Override
            protected Path withHeading(Interpolator interpolator) {
                return this;
            }
            @Override
            protected Path withModifiers(List<Modifier> list) {
                return this;
            }
        }.heading(Interpolator.linear(current.heading(), target.heading()));

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
        follower.stop();
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
