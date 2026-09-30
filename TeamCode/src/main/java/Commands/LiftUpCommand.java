package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.LiftBoxSubsystem;

/**
 * LiftUpCommand — runs the CRServo-based lift upward until the timed
 * rotation count is reached.
 *
 * The subsystem tracks elapsed time against the target rotation count.
 * This command starts the lift, waits for {@link LiftBoxSubsystem#isLiftComplete()},
 * then stops the lift.
 *
 * Designed to be used inside a {@link com.arcrobotics.ftclib.command.SequentialCommandGroup}.
 */
public class LiftUpCommand extends CommandBase {

    private final LiftBoxSubsystem liftBox;

    /**
     * @param liftBox the lift/box subsystem
     */
    public LiftUpCommand(LiftBoxSubsystem liftBox) {
        this.liftBox = liftBox;
        addRequirements(liftBox);
    }

    @Override
    public void initialize() {
        liftBox.startLiftUp();
    }

    @Override
    public void end(boolean interrupted) {
        liftBox.stopLift();
    }

    /**
     * Finishes once the subsystem reports the full rotation count is reached.
     */
    @Override
    public boolean isFinished() {
        return liftBox.isLiftComplete();
    }
}
