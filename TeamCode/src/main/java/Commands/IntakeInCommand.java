package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.IntakeSubsystem;
import Subsystems.TransferSubsystem;

import java.util.function.DoubleSupplier;

/**
 * IntakeInCommand — controls intake and transfer motors proportional to analog trigger input.
 *
 * Both the intake motor (via {@link IntakeSubsystem}) and the standalone transfer motor
 * (via {@link TransferSubsystem}) are driven at the same proportional power so game
 * elements feed smoothly from intake through transfer as the driver modulates the trigger.
 */
public class IntakeInCommand extends CommandBase {

    private final IntakeSubsystem    intakeSubsystem;
    private final TransferSubsystem  transferSubsystem;
    private final DoubleSupplier     powerSupplier;

    /**
     * @param intakeSubsystem   the intake subsystem controlling the intake motor
     * @param transferSubsystem the transfer subsystem controlling the transfer motor
     * @param powerSupplier     supplier for analog trigger value (0.0–1.0)
     */
    public IntakeInCommand(IntakeSubsystem intakeSubsystem,
                           TransferSubsystem transferSubsystem,
                           DoubleSupplier powerSupplier) {
        this.intakeSubsystem   = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.powerSupplier     = powerSupplier;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        double power = powerSupplier.getAsDouble();
        intakeSubsystem.spinIn(power);
        transferSubsystem.spinProportional(power);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
        transferSubsystem.stop();
    }

    /** Runs continuously while triggered/active. */
    @Override
    public boolean isFinished() {
        return false;
    }
}
