package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * FieldCentricDriveCommand — default command for {@link DriveSubsystem}.
 *
 * Uses the Pinpoint localizer's heading (via Pedro Pathing Follower) to
 * perform field-centric Mecanum drive.  "Forward" on the joystick always
 * pushes the robot away from the driver, regardless of the robot's current
 * orientation on the field.
 *
 * <h3>Joystick mapping (Gamepad 1)</h3>
 * <ul>
 *   <li>Left stick Y → forward/backward (field-relative)</li>
 *   <li>Left stick X → strafe left/right (field-relative)</li>
 *   <li>Right stick X → rotation (always robot-relative)</li>
 * </ul>
 *
 * Runs continuously (never finishes on its own) and is set as the
 * subsystem's default command in the OpMode.
 */
public class FieldCentricDriveCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;

    /**
     * @param driveSubsystem   the drive subsystem to control
     * @param forwardSupplier  supplier for forward/backward axis (left stick Y, negated)
     * @param strafeSupplier   supplier for strafe axis (left stick X)
     * @param rotationSupplier supplier for rotation axis (right stick X)
     */
    public FieldCentricDriveCommand(DriveSubsystem driveSubsystem,
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
        driveSubsystem.driveFieldCentric(
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
