package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.LiftBoxSubsystem;

/**
 * LiftDownCommand — runs the CRServo-based lift downward (reverse)
 * until the timed rotation count is reached.
 *
 * Mirror of {@link LiftUpCommand} for returning the lift to the bottom
 * position.
 */
public class LiftDownCommand extends CommandBase {

    private final LiftBoxSubsystem liftBox;

    /**
     * @param liftBox the lift/box subsystem
     */
    public LiftDownCommand(LiftBoxSubsystem liftBox) {
        this.liftBox = liftBox;
        addRequirements(liftBox);
    }

    @Override
    public void initialize() {
        liftBox.startLiftDown();
    }

    @Override
    public void end(boolean interrupted) {
        liftBox.stopLift();
    }

    @Override
    public boolean isFinished() {
        return liftBox.isLiftComplete();
    }
}
