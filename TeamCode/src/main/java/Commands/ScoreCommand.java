package Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import Subsystems.IntakeSubsystem;
import Subsystems.TransferSubsystem;
import Subsystems.TurretShooterSubsystem;

import util.HiveGoalTracker;

/**
 * ScoreCommand — Non-blocking one-shot scoring macro for single-driver cycle optimization.
 *
 * <h3>Purpose</h3>
 * Executes an automated intake→transfer→shoot sequence as a single button press (A button)
 * while the driver retains 100% full chassis control. Every phase runs concurrently with
 * driving, turret tracking, and all other subsystems — completely non-blocking.
 *
 * <h3>Cycle Time Optimization</h3>
 * <ul>
 *   <li>If the shooter was already pre-spinning (via Right Trigger latch), this macro skips
 *       spin-up delay and immediately feeds into the flywheel (0ms spin-up latency).</li>
 *   <li>If the shooter was pre-spinning, the flywheel remains spinning after the shot for
 *       rapid back-to-back cycling. If not, it shuts off cleanly.</li>
 *   <li>While scoring, the driver can continue translating and auto-aiming at full speed.</li>
 * </ul>
 */
public class ScoreCommand extends CommandBase {

    // ── Subsystem references ─────────────────────────────────────────────────
    private final IntakeSubsystem        intakeSubsystem;
    private final TransferSubsystem      transferSubsystem;
    private final TurretShooterSubsystem shooterSubsystem;
    private final HiveGoalTracker        goalTracker;

    // ── Timing constants (milliseconds) ──────────────────────────────────────
    /** Max time to wait for flywheel to reach target speed before forcing feed. */
    public static long SPIN_UP_TIMEOUT_MS = 350;

    /** Time for the transfer motor to push the element into the flywheel. */
    public static long FEED_MS            = 300;

    /** Post-shot clear time for element to exit before command terminates. */
    public static long POST_MS            = 150;

    // ── State machine ────────────────────────────────────────────────────────
    private enum Phase { SPIN_UP, FEEDING, POST_SHOT, DONE }
    private Phase phase;
    private long  phaseStartNanos;
    private boolean shooterWasPreSpinning = false;

    /**
     * @param intakeSubsystem    intake subsystem (required — interrupts intake collection)
     * @param transferSubsystem  transfer subsystem (called directly)
     * @param shooterSubsystem   flywheel subsystem (called directly)
     * @param goalTracker        goal tracker for distance-scaled velocity
     */
    public ScoreCommand(IntakeSubsystem intakeSubsystem,
                        TransferSubsystem transferSubsystem,
                        TurretShooterSubsystem shooterSubsystem,
                        HiveGoalTracker goalTracker) {
        this.intakeSubsystem   = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.shooterSubsystem  = shooterSubsystem;
        this.goalTracker       = goalTracker;

        // Only require intake to prevent intake commands from conflicting.
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        phase = Phase.SPIN_UP;
        phaseStartNanos = System.nanoTime();

        shooterWasPreSpinning = shooterSubsystem.isSpinning();

        // Set flywheel to distance-scaled velocity immediately
        double velocity = goalTracker.getSuggestedLaunchVelocity();
        shooterSubsystem.setTargetVelocity(velocity);

        // Stop intake to prevent double-feeding elements
        intakeSubsystem.stop();
    }

    @Override
    public void execute() {
        long elapsedMs = (System.nanoTime() - phaseStartNanos) / 1_000_000L;

        switch (phase) {
            case SPIN_UP:
                // Pre-stage element forward toward flywheel
                transferSubsystem.spinProportional(0.35);

                // Instantly advance if already at speed or timeout reached
                if (shooterSubsystem.isAtTargetSpeed() || elapsedMs >= SPIN_UP_TIMEOUT_MS) {
                    advanceTo(Phase.FEEDING);
                }
                break;

            case FEEDING:
                // Full-power feed into flywheel
                transferSubsystem.spinProportional(1.0);

                if (elapsedMs >= FEED_MS) {
                    advanceTo(Phase.POST_SHOT);
                }
                break;

            case POST_SHOT:
                // Stop transfer while element clears
                transferSubsystem.stop();

                if (elapsedMs >= POST_MS) {
                    phase = Phase.DONE;
                }
                break;

            case DONE:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Keep flywheel spinning if driver latched it on before scoring
        if (!shooterWasPreSpinning) {
            shooterSubsystem.stop();
        }
        transferSubsystem.stop();
        intakeSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return phase == Phase.DONE;
    }

    private void advanceTo(Phase next) {
        phase = next;
        phaseStartNanos = System.nanoTime();
    }
}
