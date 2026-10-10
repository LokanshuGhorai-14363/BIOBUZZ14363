package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import pedroPathing.constants.Constants;

/**
 * DriveSubsystem — Mecanum drive powered by Pedro Pathing's Follower.
 *
 * In TeleOp the Follower handles all kinematic math (inverse kinematics,
 * heading correction, etc.).  We simply feed it translation/strafe/rotation
 * vectors every loop and call {@code update()}.
 *
 * Motor configuration (goBILDA Yellow Jacket 425 RPM) is handled through
 * the Pedro Pathing constants file {@link Constants}.
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

        // Follower is created via our unified Constants class (Pedro Pathing 3.x)
        follower = Constants.createFollower(hardwareMap);
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

    /**
     * Retrieves the current pose from the Follower.
     */
    public Pose getPose() {
        return follower.getPose();
    }

    /**
     * Overwrites the robot's current pose (useful for vision relocalization).
     */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /**
     * Autonomously follows a straight line path to the target pose.
     */
    public void followPathTo(Pose target) {
        Path path = new Path(new BezierLine(
            new Point(follower.getPose()),
            new Point(target)
        ));
        path.setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading());
        follower.followPath(path);
    }

    /**
     * Checks if the Follower has completed the current path.
     */
    public boolean isPathComplete() {
        return !follower.isBusy();
    }

    /**
     * Resumes TeleOp control after an autonomous command finishes.
     */
    public void resumeTeleOp() {
        follower.startTeleopDrive();
    }
}
