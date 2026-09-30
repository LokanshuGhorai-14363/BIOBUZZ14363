package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.IntakeSubsystem;

/**
 * IntakeOutCommand — spins the intake motor in reverse (eject cargo).
 *
 * Bound to Gamepad 1 Left Bumper (button).
 *
 * Uses a fixed reverse power defined in the subsystem.
 * Automatically stops when the bumper is released.
 */
public class IntakeOutCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;

    /**
     * @param intakeSubsystem the intake subsystem
     */
    public IntakeOutCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.spinOut();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stop();
    }

    /** Runs as long as the bumper is held (whileTrue binding). */
    @Override
    public boolean isFinished() {
        return false;
    }
}
