package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * DefaultDriveCommand — default command for {@link DriveSubsystem}.
 *
 * Maps translation and rotation entirely to Gamepad 1's Left Joystick:
 * <ul>
 *   <li>Left stick Y → forward/backward</li>
 *   <li>Left stick X → strafe left/right</li>
 *   <li>Right stick X → rotation</li>
 * </ul>
 *
 * Runs continuously (never finishes on its own) and is set as the
 * subsystem's default command in the OpMode.
 */
public class DefaultDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;

    /**
     * @param driveSubsystem   the drive subsystem to control
     * @param forwardSupplier  supplier for forward/backward axis (left stick Y)
     * @param strafeSupplier   supplier for strafe axis (left stick X)
     * @param rotationSupplier supplier for rotation axis (right stick X)
     */
    public DefaultDriveCommand(DriveSubsystem driveSubsystem,
                               DoubleSupplier forwardSupplier,
                               DoubleSupplier strafeSupplier,
                               DoubleSupplier rotationSupplier) {
        this.driveSubsystem   = driveSubsystem;
        this.forwardSupplier  = forwardSupplier;
        this.strafeSupplier   = strafeSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(
                forwardSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                rotationSupplier.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    /** Never finishes — default command pattern. */
    @Override
    public boolean isFinished() {
        return false;
    }
}
