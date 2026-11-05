package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.math.Pose;
import util.HiveGoalTracker;

/**
 * VisionSubsystem — Pinpoint Odometry goal aiming and turret tracking.
 *
 * Field Specifications:
 * - Field Size: 144in x 144in (Origin (0,0) at bottom-left / Audience-left)
 * - Robot Size: 17.5in Wide x 14.5in Long
 * - Red Start Pose: (63.25, 7.25, 90°) [Back against wall at y=0, back-right corner at (X=72, Y=0)]
 * - Blue Start Pose: (80.75, 136.75, 270°) [Back against other wall at y=144, back-right corner at (X=72, Y=144)]
 * - Red Alliance Goals:  (60in, 60in) and (60in, 84in)
 * - Blue Alliance Goals: (84in, 60in) and (84in, 84in)
 */
public class VisionSubsystem extends SubsystemBase {

    private final Telemetry telemetry;
    private final TurretSubsystem turretSubsystem;
    private final DriveSubsystem driveSubsystem;

    public enum Alliance { RED, BLUE }
    private Alliance currentAlliance = Alliance.RED;
    private HiveGoalTracker goalTracker;

    // Field & Robot Dimensions (in inches)
    public static final double FIELD_SIZE_INCHES = 144.0;
    public static final double FIELD_MID_Y_INCHES = 72.0;

    public static final double ROBOT_WIDTH_INCHES = 17.5;
    public static final double ROBOT_LENGTH_INCHES = 14.5;

    // ────────────────────────── Starting Poses ──────────────────────────
    public static final Pose RED_START_POSE = new Pose(63.25, 7.25, Math.toRadians(90.0));
    public static final Pose BLUE_START_POSE = new Pose(80.75, 136.75, Math.toRadians(270.0));
    public static final Pose TOP_LEFT_RELOCALIZE_POSE = new Pose(12.0, 132.0, Math.toRadians(270.0));

    // Turret Zero / Center Offset in degrees (100.0° points straight forward relative to robot)
    public static double TURRET_ZERO_OFFSET_DEG = 100.0;

    // Invert turret rotation direction if servo turns CW instead of CCW
    public static boolean INVERT_TURRET_DIRECTION = false;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry, 
                           TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem) {
        this.telemetry = telemetry;
        this.turretSubsystem = turretSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.goalTracker = new HiveGoalTracker(currentAlliance);
    }

    public void setAlliance(Alliance alliance) {
        this.currentAlliance = alliance;
        this.goalTracker = new HiveGoalTracker(alliance);
    }

    public Alliance getAlliance() {
        return currentAlliance;
    }

    public HiveGoalTracker getGoalTracker() {
        return goalTracker;
    }

    public Pose getStartPose(Alliance alliance) {
        return (alliance == Alliance.RED) ? RED_START_POSE : BLUE_START_POSE;
    }

    public Pose getTopLeftRelocalizePose() {
        return TOP_LEFT_RELOCALIZE_POSE;
    }

    @Override
    public void periodic() {
        Pose robotPose = driveSubsystem.getPose();
        goalTracker.update(robotPose);
        double turretAngle = calculateOdometryTurretAngle();

        if (telemetry != null) {
            telemetry.addData("Alliance", currentAlliance);
            telemetry.addData("Robot Pose", String.format("(%.1fin, %.1fin, %.1f°)", 
                    robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));
            telemetry.addData("Target Goal", String.format("%s (%.1fin, %.1fin)", 
                    goalTracker.isTargetingFar() ? "FAR" : "NEAR",
                    goalTracker.getTargetX(), goalTracker.getTargetY()));
            telemetry.addData("Goal Distance (in)", "%.1f", goalTracker.getDistanceInches());
            telemetry.addData("Target Turret Angle (deg)", "%.1f", turretAngle);
        }
    }

    /**
     * Calculates the required turret angle to aim directly at the active goal using
     * Pinpoint Odometry (Pedro Pathing robot pose) synchronized with HiveGoalTracker.
     *
     * @return Required turret angle in degrees, clamped to [0, 200].
     */
    public double calculateOdometryTurretAngle() {
        Pose robotPose = driveSubsystem.getPose();

        // Target heading towards the goal in field frame
        double globalGoalAngle = goalTracker.getTargetHeadingRad();

        // Relative angle from robot's heading to goal (in radians)
        double relativeAngleRad = HiveGoalTracker.normalizeAngle(globalGoalAngle - robotPose.heading());
        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);

        if (INVERT_TURRET_DIRECTION) {
            relativeAngleDeg = -relativeAngleDeg;
        }

        // Apply turret center offset (100.0° = straight forward)
        double commandedAngle = relativeAngleDeg + TURRET_ZERO_OFFSET_DEG;

        // Clamp to physical limits [0, 200]
        return Math.max(TurretSubsystem.MIN_ANGLE_DEG, Math.min(TurretSubsystem.MAX_ANGLE_DEG, commandedAngle));
    }
}
