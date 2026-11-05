package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import Subsystems.DriveSubsystem;
import util.HiveGoalTracker;

/**
 * GoalTrackingDriveCommand — Field-centric drive with optional auto-aim.
 *
 * <h3>Behavior</h3>
 * <ul>
 *   <li>When the driver supplies rotation input (|rightStickX| &gt; deadzone),
 *       the chassis rotates purely under manual control.</li>
 *   <li>When the driver releases the rotation stick <em>and</em>
 *       {@code autoAimEnabled} is {@code true}, the command applies PID
 *       heading correction from the {@link HiveGoalTracker} to automatically
 *       face the active HIVE goal while the driver controls translation.</li>
 * </ul>
 *
 * This allows the driver to override auto-aim at any time simply by touching
 * the right stick, and resume auto-aim by releasing it.
 *
 * <p>Intended to be set as the {@link DriveSubsystem}'s default command so it
 * runs every loop iteration that no other command claims the subsystem.</p>
 */
public class GoalTrackingDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final HiveGoalTracker goalTracker;

    private final DoubleSupplier  forwardSupplier;
    private final DoubleSupplier  strafeSupplier;
    private final DoubleSupplier  rotationSupplier;
    private final BooleanSupplier autoAimToggle;

    /** Rotation stick deadzone — below this magnitude, auto-aim takes over. */
    private static final double ROTATION_DEADZONE = 0.05;

    /**
     * @param driveSubsystem   the drive subsystem to control
     * @param goalTracker      shared {@link HiveGoalTracker} (updated externally each loop)
     * @param forwardSupplier  supplier for forward axis (left stick Y, negated)
     * @param strafeSupplier   supplier for strafe axis (left stick X)
     * @param rotationSupplier supplier for rotation axis (right stick X)
     * @param autoAimToggle    returns {@code true} when auto-aim should be active;
     *                         pass {@code () -> true} for always-on
     */
    public GoalTrackingDriveCommand(DriveSubsystem driveSubsystem,
                                    HiveGoalTracker goalTracker,
                                    DoubleSupplier forwardSupplier,
                                    DoubleSupplier strafeSupplier,
                                    DoubleSupplier rotationSupplier,
                                    BooleanSupplier autoAimToggle) {
        this.driveSubsystem   = driveSubsystem;
        this.goalTracker      = goalTracker;
        this.forwardSupplier  = forwardSupplier;
        this.strafeSupplier   = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.autoAimToggle    = autoAimToggle;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double forward  = forwardSupplier.getAsDouble();
        double strafe   = strafeSupplier.getAsDouble();
        double rotation = rotationSupplier.getAsDouble();

        boolean driverRotating = Math.abs(rotation) > ROTATION_DEADZONE;

        if (!driverRotating && autoAimToggle.getAsBoolean()) {
            // Auto-aim: replace rotation with PID heading correction
            rotation = goalTracker.calculateHeadingCorrection();
        } else {
            // Manual rotation: reset PID to avoid integral buildup
            goalTracker.resetHeadingPID();
        }

        driveSubsystem.driveFieldCentric(forward, strafe, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
        goalTracker.resetHeadingPID();
    }

    /** Continuous default command — never finishes on its own. */
    @Override
    public boolean isFinished() {
        return false;
    }
}

