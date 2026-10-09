package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

/**
 * IntakeInCommand — controls intake and transfer motors with automated color sensing logic.
 *
 * <h3>Behavior</h3>
 * <ul>
 *   <li><b>Ejecting:</b> If an illegal color mix (Red + Blue simultaneously) is detected,
 *       it enters an automated purge sequence running intake and transfer motors in reverse
 *       at full power until all 4 sensors report NONE.</li>
 *   <li><b>Full Capacity:</b> If all 4 sensors detect balls (and no illegal mix), the motors
 *       automatically stop to prevent jams/overfilling.</li>
 *   <li><b>Normal Intaking:</b> Runs intake and transfer motors inward proportional
 *       to the analog trigger input.</li>
 * </ul>
 */
public class IntakeInCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier  powerSupplier;
    private boolean isEjecting = false;

    /**
     * @param intakeSubsystem the intake subsystem controlling intake + transfer motors and color sensors
     * @param powerSupplier   supplier for analog trigger value (0.0–1.0)
     */
    public IntakeInCommand(IntakeSubsystem intakeSubsystem,
                           DoubleSupplier powerSupplier) {
        this.intakeSubsystem = intakeSubsystem;
        this.powerSupplier   = powerSupplier;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        isEjecting = false;
    }

    @Override
    public void execute() {
        // 1. Check if an illegal color mix (Red + Blue) is detected to trigger purge state
        if (!isEjecting && intakeSubsystem.hasIllegalColorMix()) {
            isEjecting = true;
        }

        if (isEjecting) {
            // Eject state: run motors in REVERSE at full power until all 4 sensors report NONE
            intakeSubsystem.spinOutFull();
            if (intakeSubsystem.areAllSensorsEmpty()) {
                isEjecting = false;
                intakeSubsystem.stop();
            }
        } else if (intakeSubsystem.isFull()) {
            // Capacity limit reached (4 balls, valid mix): stop motors to prevent jamming
            intakeSubsystem.stop();
        } else {
            // Normal operation: spin intake + transfer motors inward
            intakeSubsystem.spinIn(powerSupplier.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        isEjecting = false;
        intakeSubsystem.stop();
    }

    /** Runs continuously while triggered/active. */
    @Override
    public boolean isFinished() {
        return false;
    }
}
