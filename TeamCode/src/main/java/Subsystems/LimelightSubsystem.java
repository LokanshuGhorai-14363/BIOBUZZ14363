package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

/**
 * LimelightSubsystem — Dedicated subsystem for Limelight 3A Vision Sensor.
 *
 * <h3>Robot Dimensions</h3>
 * <ul>
 *   <li>Width: 17.193 in</li>
 *   <li>Length: 14.826 in</li>
 *   <li>Height: 13.500 in</li>
 * </ul>
 *
 * <h3>Lens Mounting Offsets (Relative to Robot Center)</h3>
 * <ul>
 *   <li><b>X Offset</b>: +4.913 in (2.5 in back from front edge)</li>
 *   <li><b>Y Offset</b>: -6.0965 in (2.5 in left from right edge)</li>
 *   <li><b>Z Offset</b>: +6.500 in (6.5 in above floor)</li>
 * </ul>
 */
@Configurable
public class LimelightSubsystem extends SubsystemBase {

    private final Telemetry telemetry;
    private Limelight3A limelight;

    // Robot Dimensions (in inches)
    public static final double ROBOT_WIDTH_INCHES  = 17.193;
    public static final double ROBOT_LENGTH_INCHES = 14.826;
    public static final double ROBOT_HEIGHT_INCHES = 13.500;

    // Limelight 3A Lens Mounting Position relative to robot center (inches)
    public static double LIMELIGHT_X_OFFSET = (ROBOT_LENGTH_INCHES / 2.0) - 2.5;  // +4.913 in
    public static double LIMELIGHT_Y_OFFSET = -(ROBOT_WIDTH_INCHES / 2.0) + 2.5;  // -6.0965 in
    public static double LIMELIGHT_Z_OFFSET = 6.500;                               // +6.500 in

    public static double TURRET_ZERO_OFFSET_DEG = 100.0;
    public static boolean INVERT_TURRET_DIRECTION = true;

    public LimelightSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
        } catch (Exception e) {
            try {
                limelight = hardwareMap.get(Limelight3A.class, "limelight3a");
            } catch (Exception ignored) {}
        }

        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.start();
        }
    }

    public boolean isConnected() {
        return limelight != null;
    }

    public boolean hasValidTarget() {
        if (limelight == null) return false;
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    public LLResult getLatestResult() {
        return (limelight != null) ? limelight.getLatestResult() : null;
    }

    public double getTx() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTx() : 0.0;
    }

    public double getTy() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTy() : 0.0;
    }

    public double getTa() {
        LLResult result = getLatestResult();
        return (result != null && result.isValid()) ? result.getTa() : 0.0;
    }

    /**
     * Computes the required turret angle to aim at the vision target, applying 
     * 3D camera mounting offset transformation to convert camera-relative error to robot-center angle.
     *
     * @param defaultOdometryAngle Fallback angle if no vision target is visible.
     * @return Required turret angle in degrees, clamped to [0, 200].
     */
    public double calculateTargetTurretAngle(double defaultOdometryAngle) {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double distanceInches = 60.0; // Estimated distance to goal

            // Target position in camera frame
            double targetCamX = distanceInches * Math.sin(Math.toRadians(tx));
            double targetCamY = distanceInches * Math.cos(Math.toRadians(tx));

            // Transform target position to robot center frame using exact mounting offsets
            double targetRobotX = targetCamY + LIMELIGHT_X_OFFSET;
            double targetRobotY = targetCamX + LIMELIGHT_Y_OFFSET;

            double correctedAngleRad = Math.atan2(targetRobotY, targetRobotX);
            double relativeAngleDeg = Math.toDegrees(correctedAngleRad);

            if (INVERT_TURRET_DIRECTION) {
                relativeAngleDeg = -relativeAngleDeg;
            }

            double commandedAngle = relativeAngleDeg + TURRET_ZERO_OFFSET_DEG;
            return Math.max(TurretSubsystem.MIN_ANGLE_DEG, Math.min(TurretSubsystem.MAX_ANGLE_DEG, commandedAngle));
        }

        return defaultOdometryAngle;
    }

    @Override
    public void periodic() {
        if (telemetry != null) {
            boolean connected = isConnected();
            boolean valid = hasValidTarget();
            telemetry.addData("Limelight 3A Connected", connected);
            telemetry.addData("Limelight Target Valid", valid);
            if (valid) {
                telemetry.addData("Limelight Tx / Ty", String.format(Locale.US, "%.2f°, %.2f°", getTx(), getTy()));
            }
        }
    }
}
