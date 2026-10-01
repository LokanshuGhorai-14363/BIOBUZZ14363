package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.pedropathing.localization.Pose;

import java.util.Arrays;
import java.util.List;

/**
 * VisionSubsystem — Limelight 3G integration for dynamic turret tracking and relocalization.
 *
 * The Limelight is mounted on a rotating turret, 84.125 mm to the LEFT of the center,
 * with a 21-degree upward pitch.
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

    // Field Y-coordinate threshold for top/bottom half (in inches, assuming 0-144 field, middle is 72)
    public static final double FIELD_MID_Y_INCHES = 72.0;

    // Target Tag IDs
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

    @Override
    public void periodic() {
        lastResult = limelight.getLatestResult();
        
        if (lastResult != null && lastResult.isValid()) {
            telemetry.addData("LL Targets", lastResult.getTa());
            telemetry.addData("LL tx", lastResult.getTx());
        } else {
            telemetry.addData("LL Targets", "None");
        }
    }

    /**
     * Determines which tags to prioritize based on Alliance and robot's Y position.
     * @return List of prioritized Tag IDs.
     */
    public List<Integer> getPrioritizedTags() {
        Pose robotPose = driveSubsystem.getPose();
        boolean isTopHalf = robotPose.getY() > FIELD_MID_Y_INCHES;

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

        // Botpose from Limelight (assuming LL is configured with 0,0,0 camera pose so it returns camera's field pose)
        // Convert to inches (PedroPathing uses inches typically)
        double camX = botpose3d.getPosition().x * 39.3701;
        double camY = botpose3d.getPosition().y * 39.3701;
        double camHeading = botpose3d.getOrientation().getYaw(); 

        // Turret angle relative to robot chassis
        double turretAngleRad = Math.toRadians(turretSubsystem.getCurrentAngle());
        
        // Robot Heading = Camera Heading - Turret Angle
        double robotHeading = camHeading - turretAngleRad;

        // Offset in inches (84.125 mm to the left of turret center)
        double offsetInches = Y_OFFSET_MM / 25.4;
        
        // Camera is 'offsetInches' to the left of the turret center (along the turret's local Y axis).
        // Turret's global heading is 'camHeading'.
        // So turret center is at:
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
            return turretSubsystem.getCurrentAngle();
        }

        // Ideally, we'd filter for the prioritized tag ID here.
        // For simplicity, we'll use the primary target's tx/ty.
        double tx = lastResult.getTx(); // Horizontal angle offset
        double ty = lastResult.getTy(); // Vertical angle offset
        
        // Calculate horizontal distance to target (d).
        // Assuming a known target height. We can also use camera pose in target space if available.
        // Using basic trigonometry with the 21 degree pitch:
        // d = (TargetHeight - CameraHeight) / tan(pitch + ty)
        // However, a simpler way is to just use tx to adjust the turret, but we need to account for the Y offset.
        
        // If we only have tx, and we know the Y offset is 84.125mm:
        // Assuming an average distance to the tag of 1000mm.
        double assumedDistanceMm = 1000.0; 
        
        // Target's local coordinates relative to the camera
        double tagX_cam = assumedDistanceMm * Math.sin(Math.toRadians(tx));
        double tagY_cam = assumedDistanceMm * Math.cos(Math.toRadians(tx));
        
        // Target's coordinates relative to the turret center
        // Camera is at (0, 84.125) relative to turret
        double tagX_turret = tagX_cam;
        double tagY_turret = tagY_cam + Y_OFFSET_MM;
        
        // Required angle adjustment
        double angleAdjustment = Math.toDegrees(Math.atan2(tagX_turret, tagY_turret));
        
        return turretSubsystem.getCurrentAngle() + angleAdjustment;
    }
}
