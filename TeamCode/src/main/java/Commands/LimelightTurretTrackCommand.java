package Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Subsystems.LimelightSubsystem;
import Subsystems.TurretSubsystem;
import Subsystems.VisionSubsystem;

/**
 * LimelightTurretTrackCommand — Tracks targets using Limelight 3A vision with fallback to Odometry.
 */
public class LimelightTurretTrackCommand extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final BooleanSupplier manualModeSupplier;
    private final DoubleSupplier manualStickXSupplier;

    public static final double MAX_TURRET_SPEED_DEG_PER_SEC = 250.0;
    private final ElapsedTime timer = new ElapsedTime();

    public LimelightTurretTrackCommand(TurretSubsystem turretSubsystem,
                                       LimelightSubsystem limelightSubsystem,
                                       VisionSubsystem visionSubsystem,
                                       BooleanSupplier manualModeSupplier,
                                       DoubleSupplier manualStickXSupplier) {
        this.turretSubsystem      = turretSubsystem;
        this.limelightSubsystem   = limelightSubsystem;
        this.visionSubsystem      = visionSubsystem;
        this.manualModeSupplier   = manualModeSupplier;
        this.manualStickXSupplier = manualStickXSupplier;

        addRequirements(turretSubsystem);
        if (limelightSubsystem != null) {
            addRequirements(limelightSubsystem);
        }
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        double rawDt = timer.seconds();
        timer.reset();
        double dt = (rawDt > 0.1 || rawDt <= 0) ? 0.02 : rawDt;

        if (manualModeSupplier.getAsBoolean()) {
            double stickX = manualStickXSupplier.getAsDouble();
            if (Math.abs(stickX) < 0.05) {
                stickX = 0.0;
            }
            double currentAngle = turretSubsystem.getCurrentAngle();
            double newTargetAngle = currentAngle + (stickX * MAX_TURRET_SPEED_DEG_PER_SEC * dt);
            turretSubsystem.setAngle(newTargetAngle);
        } else {
            double defaultOdometryAngle = (visionSubsystem != null) ? visionSubsystem.calculateOdometryTurretAngle() : VisionSubsystem.TURRET_ZERO_OFFSET_DEG;
            double targetAngle = (limelightSubsystem != null)
                    ? limelightSubsystem.calculateTargetTurretAngle(defaultOdometryAngle)
                    : defaultOdometryAngle;
            turretSubsystem.setAngle(targetAngle);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
