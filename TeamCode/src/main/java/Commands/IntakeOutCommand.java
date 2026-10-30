package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

/**
 * IntakeOutCommand — spins intake and transfer motors in reverse proportional to analog input.
 */
public class IntakeOutCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier  powerSupplier;

    /**
     * @param intakeSubsystem the intake subsystem controlling intake + transfer motors
     * @param powerSupplier   supplier for analog input power (0.0–1.0)
     */
    public IntakeOutCommand(IntakeSubsystem intakeSubsystem,
                            DoubleSupplier powerSupplier) {
        this.intakeSubsystem = intakeSubsystem;
        this.powerSupplier   = powerSupplier;

        addRequirements(intakeSubsystem);
    }

    /**
     * Default constructor for bumper/button bindings using full power (1.0).
     *
     * @param intakeSubsystem the intake subsystem controlling intake + transfer motors
     */
    public IntakeOutCommand(IntakeSubsystem intakeSubsystem) {
        this(intakeSubsystem, () -> 1.0);
    }

    @Override
    public void execute() {
        intakeSubsystem.spinOut(powerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    /** Runs as long as the bumper/button is held. */
    @Override
    public boolean isFinished() {
        return false;
    }
}
