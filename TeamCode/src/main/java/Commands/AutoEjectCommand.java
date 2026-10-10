package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.IntakeSubsystem;

/**
 * AutoEjectCommand — runs intake and transfer motors in full reverse
 * until all 4 color sensors report empty (NONE).
 */
public class AutoEjectCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    /**
     * @param intakeSubsystem the intake subsystem controlling intake + transfer motors and color sensors
     */
    public AutoEjectCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.spinOutFull();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.areAllSensorsEmpty();
    }
}
