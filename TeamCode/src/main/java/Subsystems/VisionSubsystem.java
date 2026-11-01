package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.math.Pose;

/**
 * VisionSubsystem — Pinpoint Odometry goal aiming and turret tracking.
 *
 * Field Specifications:
 * - Field Size: 144in x 144in (Origin (0,0) at bottom-left / Audience-left)
 * - Robot Size: 17.5in Wide x 14.5in Long
 * - Red Start Pose: (63.25, 7.25, 90°) [Back against wall at y=0, back-right corner at (X=72, Y=0)]
 * - Blue Start Pose: (80.75, 136.75, 270°) [Back against other wall at y=144, back-right corner at (X=72, Y=144)]
 * - Top-Left Relocalize Pose: (12.0, 132.0, 270°) [Top-Left Tile [0,24]x[120,144]]
 * - Red Alliance Goals:  (60in, 60in) and (60in, 84in)
 * - Blue Alliance Goals: (84in, 60in) and (84in, 84in)
 */
public class VisionSubsystem extends SubsystemBase {

    private final Telemetry telemetry;
    private final TurretSubsystem turretSubsystem;
    private final DriveSubsystem driveSubsystem;

    public enum Alliance { RED, BLUE }
    private Alliance currentAlliance = Alliance.RED;

    // Field & Robot Dimensions (in inches)
    public static final double FIELD_SIZE_INCHES = 144.0;
    public static final double FIELD_MID_Y_INCHES = 72.0;
    public static final double FIELD_MID_X_INCHES = 72.0;

    public static final double ROBOT_WIDTH_INCHES = 17.5;
    public static final double ROBOT_LENGTH_INCHES = 14.5;

    // ────────────────────────── Starting Poses ──────────────────────────
    // Red Start: Back of robot against bottom wall (y=0), back-right corner at (X=72, Y=0).
    // Facing forward into field (+Y direction, heading 90°).
    // Center X = 72.0 - (17.5 / 2) = 63.25, Center Y = 0 + (14.5 / 2) = 7.25.
    public static final Pose RED_START_POSE = new Pose(63.25, 7.25, Math.toRadians(90.0));

    // Blue Start: Back of robot against top wall (y=144), back-right corner at (X=72, Y=144).
    // Facing forward into field (-Y direction, heading 270° / -90°).
    // Center X = 72.0 + (17.5 / 2) = 80.75, Center Y = 144.0 - (14.5 / 2) = 136.75.
    public static final Pose BLUE_START_POSE = new Pose(80.75, 136.75, Math.toRadians(270.0));

    // ───────────────────── Relocalization Poses ─────────────────────────
    // Top-Left Square: Tile spanning X in [0, 24], Y in [120, 144].
    // Center of tile is (12.0, 132.0). Default heading is facing into field (South, 270°).
    public static final Pose TOP_LEFT_RELOCALIZE_POSE = new Pose(12.0, 132.0, Math.toRadians(270.0));

    // ──────────────────────── Goal Coordinates ──────────────────────────
    // Red Alliance Goals: Column 3 (W-X seam) -> (60in, 60in) and (60in, 84in)
    public static final double RED_GOAL_X = 60.0;
    public static final double RED_GOAL_Y1 = 60.0; // Lower Red Goal (Row 3)
    public static final double RED_GOAL_Y2 = 84.0; // Upper Red Goal (Row 4)

    // Blue Alliance Goals: Column 4 (X-Y seam) -> (84in, 60in) and (84in, 84in)
    public static final double BLUE_GOAL_X = 84.0;
    public static final double BLUE_GOAL_Y1 = 60.0; // Lower Blue Goal (Row 3)
    public static final double BLUE_GOAL_Y2 = 84.0; // Upper Blue Goal (Row 4)

    // Turret Zero / Center Offset in degrees (0.0 if turret servo 0° is facing forward, or 110.0 if centered)
    public static double TURRET_ZERO_OFFSET_DEG = 0.0;

    // Pre-allocated array to prevent object creation in active loops
    private final double[] activeTargetGoal = new double[2];

    // Throttle telemetry updates to save loop time
    private long lastTelemetryTime = 0;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry, 
                           TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem) {
        this.telemetry = telemetry;
        this.turretSubsystem = turretSubsystem;
        this.driveSubsystem = driveSubsystem;
    }

    public void setAlliance(Alliance alliance) {
        this.currentAlliance = alliance;
    }

    public Alliance getAlliance() {
        return currentAlliance;
    }

    public Pose getStartPose(Alliance alliance) {
        return (alliance == Alliance.RED) ? RED_START_POSE : BLUE_START_POSE;
    }

    public Pose getTopLeftRelocalizePose() {
        return TOP_LEFT_RELOCALIZE_POSE;
    }

    @Override
    public void periodic() {
        // Only format/push telemetry every 100ms to prevent blocking the thread
        if (System.currentTimeMillis() - lastTelemetryTime > 100) {
            lastTelemetryTime = System.currentTimeMillis();

            // Output Odometry Aiming Info to Telemetry
            Pose robotPose = driveSubsystem.getPose();
            updateTargetGoalCoordinates();
            double distance = Math.hypot(activeTargetGoal[0] - robotPose.x(), activeTargetGoal[1] - robotPose.y());
            double turretAngle = calculateOdometryTurretAngle();

            telemetry.addData("Alliance", currentAlliance);
            telemetry.addData("Robot Pose", String.format("(%.1f, %.1f, %.1f°)", 
                    robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));
            telemetry.addData("Target Goal", "( " + activeTargetGoal[0] + "in, " + activeTargetGoal[1] + "in )");
            telemetry.addData("Goal Distance", distance);
            telemetry.addData("Target Turret Angle", turretAngle);
        }
    }

    /**
     * Determines the active target goal coordinates (X, Y in inches) based on alliance
     * and the robot's current Y position, caching it in activeTargetGoal.
     */
    public void updateTargetGoalCoordinates() {
        Pose robotPose = driveSubsystem.getPose();
        double robotY = robotPose.y();

        if (currentAlliance == Alliance.RED) {
            activeTargetGoal[0] = RED_GOAL_X;
            activeTargetGoal[1] = (robotY > FIELD_MID_Y_INCHES) ? RED_GOAL_Y2 : RED_GOAL_Y1;
        } else {
            activeTargetGoal[0] = BLUE_GOAL_X;
            activeTargetGoal[1] = (robotY > FIELD_MID_Y_INCHES) ? BLUE_GOAL_Y2 : BLUE_GOAL_Y1;
        }
    }

    /**
     * Calculates the required turret angle to aim directly at the active goal using
     * Pinpoint Odometry (Pedro Pathing robot pose).
     *
     * @return Required turret angle in degrees, clamped to [MIN_ANGLE_DEG, MAX_ANGLE_DEG].
     */
    public double calculateOdometryTurretAngle() {
        Pose robotPose = driveSubsystem.getPose();
        updateTargetGoalCoordinates();

        double dx = activeTargetGoal[0] - robotPose.x();
        double dy = activeTargetGoal[1] - robotPose.y();

        // Field heading angle towards the goal (in radians)
        double globalGoalAngle = Math.atan2(dy, dx);

        // Relative angle from robot's heading to goal
        double relativeAngleRad = globalGoalAngle - robotPose.heading();

        // Normalize to [-PI, PI] efficiently
        relativeAngleRad = (relativeAngleRad + Math.PI) % (2 * Math.PI) - Math.PI;
        if (relativeAngleRad < -Math.PI) {
            relativeAngleRad += 2 * Math.PI;
        }

        double relativeAngleDeg = Math.toDegrees(relativeAngleRad);

        // Apply any mechanical turret zero offset
        double commandedAngle = relativeAngleDeg + TURRET_ZERO_OFFSET_DEG;

        // Clamp to physical limits [0, 220] instead of wrapping around to 360°
        if (commandedAngle < TurretSubsystem.MIN_ANGLE_DEG) {
            commandedAngle = TurretSubsystem.MIN_ANGLE_DEG;
        } else if (commandedAngle > TurretSubsystem.MAX_ANGLE_DEG) {
            commandedAngle = TurretSubsystem.MAX_ANGLE_DEG;
        }

        return commandedAngle;
    }
}
