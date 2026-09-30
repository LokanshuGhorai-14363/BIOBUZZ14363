package Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import Subsystems.LiftBoxSubsystem;

/**
 * LiftDumpSequence — the complete "score cargo" macro.
 *
 * Executes in strict order:
 * <ol>
 *   <li>{@link InstantCommand}: Close the box door to 50° to secure cargo.</li>
 *   <li>{@link WaitCommand}: Brief 300 ms pause for the door servo to settle.</li>
 *   <li>{@link LiftUpCommand}: Drive both CRServos in sync at max power
 *       until 20 rotations elapse (timed via the subsystem).</li>
 *   <li>{@link InstantCommand}: Rotate the box pivot 90° to dump.</li>
 *   <li>{@link WaitCommand}: Hold dump position for 1 second.</li>
 *   <li>{@link InstantCommand}: Return box pivot to home.</li>
 *   <li>{@link LiftDownCommand}: Lower the lift back to the bottom.</li>
 *   <li>{@link InstantCommand}: Open the door back to 0° (default state).</li>
 * </ol>
 *
 * Triggered by Gamepad 1 D-pad Up (or Y/Triangle).
 */
public class LiftDumpSequence extends SequentialCommandGroup {

    /**
     * @param liftBox the shared lift/box subsystem
     */
    public LiftDumpSequence(LiftBoxSubsystem liftBox) {
        addCommands(
                // 1. Close the door to lock in the cargo
                new InstantCommand(liftBox::closeDoor, liftBox),

                // 2. Small wait for the door servo to reach position
                new WaitCommand(300),

                // 3. Raise the lift — both CRServos in sync for 20 rotations
                new LiftUpCommand(liftBox),

                // 4. Dump — rotate the box pivot 90°
                new InstantCommand(liftBox::pivotDump, liftBox),

                // 5. Hold dump position for 1 second
                new WaitCommand(1000),

                // 6. Return pivot to home
                new InstantCommand(liftBox::pivotHome, liftBox),

                // 7. Lower the lift back down
                new LiftDownCommand(liftBox),

                // 8. Open the door back to default
                new InstantCommand(liftBox::openDoor, liftBox)
        );
    }
}
