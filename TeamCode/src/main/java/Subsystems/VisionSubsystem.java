package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.pedropathing.math.Pose;
import java.util.Arrays;
import java.util.List;

/**
 * VisionSubsystem — Limelight 3A integration & Pinpoint Odometry aiming for dynamic turret tracking.
 *
 * The Limelight is mounted on a rotating turret, 84.125 mm to the LEFT of the center,
 * with a 21-degree upward pitch.
 *
 * Field Specifications:
 * - Field Size: 144in x 144in (Origin (0,0) at bottom-left)
 * - Red Alliance Goals:  (60in, 60in) and (60in, 84in)
 * - Blue Alliance Goals: (84in, 60in) and (84in, 84in)
 */
public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private final Telemetry telemetry;
    private final TurretSubsystem turretSubsystem;
    private final DriveSubsystem driveSubsystem;

    public enum Alliance { RED, BLUE }
    private Alliance currentAlliance = Alliance.RED;

    // Y-Offset of Limelight from turret center (in mm)
    public static final double Y_OFFSET_MM = 84.125;
    // Pitch angle of the Limelight (in degrees)
    public static final double PITCH_ANGLE_DEG = 21.0;

    // Field & Goal Coordinates (in inches)
    public static final double FIELD_SIZE_INCHES = 144.0;
    public static final double FIELD_MID_Y_INCHES = 72.0;

    // Red Alliance Goals: (60in, 60in) and (60in, 84in)
    public static final double RED_GOAL_X = 60.0;
    public static final double RED_GOAL_Y1 = 60.0;
    public static final double RED_GOAL_Y2 = 84.0;

    // Blue Alliance Goals: (84in, 60in) and (84in, 84in)
    public static final double BLUE_GOAL_X = 84.0;
    public static final double BLUE_GOAL_Y1 = 60.0;
    public static final double BLUE_GOAL_Y2 = 84.0;

    // Target Tag IDs for Limelight fallback
    private final List<Integer> RED_SCORING_TAGS = Arrays.asList(30, 31, 32, 33);
    private final List<Integer> RED_AUDIENCE_TAGS = Arrays.asList(34, 35, 36, 37);
    private final List<Integer> BLUE_SCORING_TAGS = Arrays.asList(42, 43, 44, 45);
    private final List<Integer> BLUE_AUDIENCE_TAGS = Arrays.asList(38, 39, 40, 41);

    private LLResult lastResult;

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry, 
                           TurretSubsystem turretSubsystem, DriveSubsystem driveSubsystem) {
        this.telemetry = telemetry;
        this.turretSubsystem = turretSubsystem;
        this.driveSubsystem = driveSubsystem;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void setAlliance(Alliance alliance) {
        this.currentAlliance = alliance;
    }

    public Alliance getAlliance() {
        return currentAlliance;
    }

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
        
        if (lastResult != null && lastResult.isValid()) {
            telemetry.addData("LL Targets", lastResult.getTa());
            telemetry.addData("LL tx", lastResult.getTx());
        } else {
            telemetry.addData("LL Targets", "None");
        }

        // Output Odometry Aiming Info to Telemetry
        Pose robotPose = driveSubsystem.getPose();
        double[] targetGoal = getTargetGoalCoordinates();
        double distance = Math.hypot(targetGoal[0] - robotPose.x(), targetGoal[1] - robotPose.y());
        double turretAngle = calculateOdometryTurretAngle();

        telemetry.addData("Alliance", currentAlliance);
        telemetry.addData("Target Goal", "(%.1fin, %.1fin)", targetGoal[0], targetGoal[1]);
        telemetry.addData("Goal Distance", "%.2fin", distance);
        telemetry.addData("Target Turret Angle", "%.2f°", turretAngle);
    }

    /**
     * Determines the active target goal coordinates (X, Y in inches) based on alliance
     * and the robot's current Y position.
     *
     * @return double array {goalX, goalY}
     */
    public double[] getTargetGoalCoordinates() {
        Pose robotPose = driveSubsystem.getPose();
        double robotY = robotPose.y();

        double goalX;
        double goalY;

        if (currentAlliance == Alliance.RED) {
            goalX = RED_GOAL_X;
            goalY = (robotY > FIELD_MID_Y_INCHES) ? RED_GOAL_Y2 : RED_GOAL_Y1;
        } else {
            goalX = BLUE_GOAL_X;
            goalY = (robotY > FIELD_MID_Y_INCHES) ? BLUE_GOAL_Y2 : BLUE_GOAL_Y1;
        }

        return new double[]{goalX, goalY};
    }

    /**
     * Calculates the required turret angle to aim directly at the active goal using
     * Pinpoint Odometry (Pedro Pathing robot pose).
     *
     * @return Required turret angle in degrees.
     */
    public double calculateOdometryTurretAngle() {
        Pose robotPose = driveSubsystem.getPose();
        double[] goalCoords = getTargetGoalCoordinates();

        double dx = goalCoords[0] - robotPose.x();
        double dy = goalCoords[1] - robotPose.y();

        // Field heading angle towards the goal (in radians)
        double globalGoalAngle = Math.atan2(dy, dx);

        // Relative angle from robot's heading to goal
        double relativeAngleRad = globalGoalAngle - robotPose.heading();

        // Normalize to [-PI, PI]
        while (relativeAngleRad > Math.PI) relativeAngleRad -= 2 * Math.PI;
        while (relativeAngleRad < -Math.PI) relativeAngleRad += 2 * Math.PI;

        double targetAngleDeg = Math.toDegrees(relativeAngleRad);

        // If target angle is negative, convert to positive [0, 360] range
        if (targetAngleDeg < 0) {
            targetAngleDeg += 360.0;
        }

        return targetAngleDeg;
    }

    /**
     * Determines which tags to prioritize based on Alliance and robot's Y position.
     * @return List of prioritized Tag IDs.
     */
    public List<Integer> getPrioritizedTags() {
        Pose robotPose = driveSubsystem.getPose();
        boolean isTopHalf = robotPose.y() > FIELD_MID_Y_INCHES;

        if (currentAlliance == Alliance.RED) {
            return isTopHalf ? RED_SCORING_TAGS : RED_AUDIENCE_TAGS;
        } else {
            return isTopHalf ? BLUE_SCORING_TAGS : BLUE_AUDIENCE_TAGS;
        }
    }

    /**
     * Calculates the robot's true center Pose for pedroPathing based on the Limelight's botpose.
     * Applies the dynamic offset based on the current turret angle.
     * 
     * @return Corrected Pose of the robot center, or null if no valid botpose.
     */
    public Pose getCorrectedBotPose() {
        if (lastResult == null || !lastResult.isValid()) return null;

        Pose3D botpose3d = lastResult.getBotpose();
        if (botpose3d == null) return null;

        double camX = botpose3d.getPosition().x * 39.3701;
        double camY = botpose3d.getPosition().y * 39.3701;
        double camHeading = botpose3d.getOrientation().getYaw(); 

        double turretAngleRad = Math.toRadians(turretSubsystem.getCurrentAngle());
        double robotHeading = camHeading - turretAngleRad;

        double offsetInches = Y_OFFSET_MM / 25.4;
        
        double robotX = camX - offsetInches * Math.sin(robotHeading + turretAngleRad);
        double robotY = camY + offsetInches * Math.cos(robotHeading + turretAngleRad);
        
        return new Pose(robotX, robotY, robotHeading);
    }

    /**
     * Calculates the required turret angle to track the optimal prioritized tag, 
     * accounting for the Limelight's physical offset and pitch.
     * 
     * @return The target angle in degrees for the TurretSubsystem, or current angle if no target.
     */
    public double calculateRequiredTurretAngle() {
        if (lastResult == null || !lastResult.isValid()) {
            return calculateOdometryTurretAngle();
        }

        double tx = lastResult.getTx();
        double ty = lastResult.getTy();
        
        double assumedDistanceMm = 1000.0; 
        
        double tagX_cam = assumedDistanceMm * Math.sin(Math.toRadians(tx));
        double tagY_cam = assumedDistanceMm * Math.cos(Math.toRadians(tx));
        
        double tagX_turret = tagX_cam;
        double tagY_turret = tagY_cam + Y_OFFSET_MM;
        
        double angleAdjustment = Math.toDegrees(Math.atan2(tagX_turret, tagY_turret));
        
        return turretSubsystem.getCurrentAngle() + angleAdjustment;
    }
}
