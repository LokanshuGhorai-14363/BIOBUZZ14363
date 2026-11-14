package Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Subsystems.TurretSubsystem;
import Subsystems.VisionSubsystem;

/**
 * TurretTrackGoalCommand — Continuous command for TurretSubsystem.
 *
 * Supports dual operating modes:
 * <ol>
 *   <li><b>Odometry Goal Tracking (Auto)</b>: Continuously computes and sets the turret angle
 *       to face the active alliance goal using Pinpoint odometry via {@link VisionSubsystem}.</li>
 *   <li><b>Manual Control</b>: Allows the co-driver (Gamepad 2) to manually steer the turret angle
 *       using Left Stick X. The analog stick displacement directly scales speed and direction
 *       up to maximum speed (250°/s) for ultra-fast tracking response.</li>
 * </ol>
 */
public class TurretTrackGoalCommand extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final BooleanSupplier manualModeSupplier;
    private final DoubleSupplier manualStickXSupplier;

    /** Maximum physical turret sweep speed in degrees per second for maximum responsiveness. */
    public static final double MAX_TURRET_SPEED_DEG_PER_SEC = 250.0;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Legacy constructor (defaults to always-auto odometry tracking).
     */
    public TurretTrackGoalCommand(TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem) {
        this(turretSubsystem, visionSubsystem, () -> false, () -> 0.0);
    }

    /**
     * Dual-mode constructor supporting co-driver manual override with analog speed scaling.
     *
     * @param turretSubsystem       the turret subsystem to command
     * @param visionSubsystem       the vision subsystem for odometry goal angle calculation
     * @param manualModeSupplier    supplier returning true when manual control is active
     * @param manualStickXSupplier  supplier for manual rotation axis (Gamepad 2 Left Stick X)
     */
    public TurretTrackGoalCommand(TurretSubsystem turretSubsystem,
                                  VisionSubsystem visionSubsystem,
                                  BooleanSupplier manualModeSupplier,
                                  DoubleSupplier manualStickXSupplier) {
        this.turretSubsystem      = turretSubsystem;
        this.visionSubsystem      = visionSubsystem;
        this.manualModeSupplier   = manualModeSupplier;
        this.manualStickXSupplier = manualStickXSupplier;

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        double rawDt = timer.seconds();
        timer.reset();

        // Safety cap dt to prevent large jumps on initial execution or lag spikes
        double dt = (rawDt > 0.1 || rawDt <= 0) ? 0.02 : rawDt;

        if (manualModeSupplier.getAsBoolean()) {
            double stickX = manualStickXSupplier.getAsDouble();

            // Deadzone filter to prevent unwanted stick drift
            if (Math.abs(stickX) < 0.05) {
                stickX = 0.0;
            }

            // Analog stick displacement (-1.0 to 1.0) directly scales turret speed and direction up to max speed
            double currentAngle = turretSubsystem.getCurrentAngle();
            double newTargetAngle = currentAngle + (stickX * MAX_TURRET_SPEED_DEG_PER_SEC * dt);
            turretSubsystem.setAngle(newTargetAngle);
        } else {
            double targetAngle = visionSubsystem.calculateOdometryTurretAngle();
            turretSubsystem.setAngle(targetAngle);
        }
    }

    /**
     * Continuous command — runs indefinitely while set as default command.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
