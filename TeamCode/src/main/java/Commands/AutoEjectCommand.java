package Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import Subsystems.IntakeSubsystem;

/**
 * AutoEjectCommand — timed reverse purge of intake and transfer motors for 1 second.
 */
public class AutoEjectCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final ElapsedTime     timer = new ElapsedTime();
    public static final double    PURGE_DURATION_MS = 1000.0;

    /**
     * @param intakeSubsystem the intake subsystem controlling intake + transfer motors
     */
    public AutoEjectCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
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
        return timer.milliseconds() >= PURGE_DURATION_MS;
    }
}
