package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

/**
 * IntakeInCommand — spins the intake motor inward (collect cargo).
 *
 * Bound to Gamepad 1 Left Trigger (analog axis).  The trigger value
 * is fed directly as the power multiplier so the driver has
 * proportional speed control.
 *
 * Automatically stops the intake when the command ends (trigger released).
 */
public class IntakeInCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier  powerSupplier;

    /**
     * @param intakeSubsystem the intake subsystem
     * @param powerSupplier   supplier for analog trigger value (0.0–1.0)
     */
    public IntakeInCommand(IntakeSubsystem intakeSubsystem,
                           DoubleSupplier powerSupplier) {
        this.intakeSubsystem = intakeSubsystem;
        this.powerSupplier   = powerSupplier;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.spinIn(powerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    /** Runs as long as the trigger is held (whileTrue binding). */
    @Override
    public boolean isFinished() {
        return false;
    }
}
