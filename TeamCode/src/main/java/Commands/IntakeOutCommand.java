package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.IntakeSubsystem;
import Subsystems.TransferSubsystem;

import java.util.function.DoubleSupplier;

/**
 * IntakeOutCommand — spins intake and transfer motors in reverse proportional to analog input.
 *
 * Both the intake motor (via {@link IntakeSubsystem}) and the standalone transfer motor
 * (via {@link TransferSubsystem}) are driven in reverse at the same proportional power.
 */
public class IntakeOutCommand extends CommandBase {

    private final IntakeSubsystem    intakeSubsystem;
    private final TransferSubsystem  transferSubsystem;
    private final DoubleSupplier     powerSupplier;

    /**
     * @param intakeSubsystem   the intake subsystem controlling the intake motor
     * @param transferSubsystem the transfer subsystem controlling the transfer motor
     * @param powerSupplier     supplier for analog input power (0.0–1.0)
     */
    public IntakeOutCommand(IntakeSubsystem intakeSubsystem,
                            TransferSubsystem transferSubsystem,
                            DoubleSupplier powerSupplier) {
        this.intakeSubsystem   = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.powerSupplier     = powerSupplier;

        addRequirements(intakeSubsystem);
    }

    /**
     * Default constructor for bumper/button bindings using full reverse power (1.0).
     *
     * @param intakeSubsystem   the intake subsystem controlling the intake motor
     * @param transferSubsystem the transfer subsystem controlling the transfer motor
     */
    public IntakeOutCommand(IntakeSubsystem intakeSubsystem,
                            TransferSubsystem transferSubsystem) {
        this(intakeSubsystem, transferSubsystem, () -> 1.0);
    }

    @Override
    public void execute() {
        double power = powerSupplier.getAsDouble();
        intakeSubsystem.spinOut(power);
        transferSubsystem.spinProportionalReverse(power);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        transferSubsystem.stop();
    }

    /** Runs as long as the bumper/button is held. */
    @Override
    public boolean isFinished() {
        return false;
    }
}
