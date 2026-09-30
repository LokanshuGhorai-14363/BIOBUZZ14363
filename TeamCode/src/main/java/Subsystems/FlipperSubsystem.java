package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * FlipperSubsystem — automated ball-detection and flip into turret.
 *
 * <h3>Hardware</h3>
 * <ul>
 *   <li>{@code flipperSensor} — REV 2m Distance Sensor.</li>
 *   <li>{@code flipperServo}  — Axon MAX Mark 2 servo.</li>
 * </ul>
 *
 * <h3>Behavior</h3>
 * If the distance sensor reads &lt; {@link #DETECT_THRESHOLD_CM},
 * the servo flips the ball into the turret, holds momentarily, then
 * returns to rest.  A cooldown timer prevents rapid re-triggering.
 *
 * This logic runs inside {@link #periodic()} so it behaves as a
 * background task without requiring a dedicated Command.
 */
public class FlipperSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DistanceSensor flipperSensor;
    private final Servo          flipperServo;
    private final Telemetry      telemetry;

    // ── Constants ─────────────────────────────────────────────────────────────
    /** Distance threshold in cm that indicates a ball is present. */
    public static final double DETECT_THRESHOLD_CM = 5.0;
    /** Servo position when at rest (not flipping). */
    public static final double REST_POSITION       = 0.0;
    /** Servo position when actively flipping a ball. */
    public static final double FLIP_POSITION        = 1.0;
    /** Time in ms to hold the flip position before returning to rest. */
    public static final long   FLIP_HOLD_MS         = 500;
    /** Cooldown in ms after a flip before another can trigger. */
    public static final long   COOLDOWN_MS           = 1000;

    // ── State machine ─────────────────────────────────────────────────────────
    private enum FlipperState { IDLE, FLIPPING, RETURNING }
    private FlipperState state = FlipperState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     */
    public FlipperSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        flipperSensor = hardwareMap.get(DistanceSensor.class, "flipperSensor");
        flipperServo  = hardwareMap.get(Servo.class, "flipperServo");

        flipperServo.setPosition(REST_POSITION);
    }

    // ───────────────────────────── Manual overrides (optional) ─────────────────

    /** Force the flipper to the flip position. */
    public void flip() {
        flipperServo.setPosition(FLIP_POSITION);
    }

    /** Force the flipper to the rest position. */
    public void rest() {
        flipperServo.setPosition(REST_POSITION);
    }

    /**
     * @return current distance reading in centimeters.
     */
    public double getDistanceCm() {
        return flipperSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * @return {@code true} if a ball is detected within threshold distance.
     */
    public boolean isBallDetected() {
        return getDistanceCm() < DETECT_THRESHOLD_CM;
    }

    // ───────────────────────────── Background automation ──────────────────────

    /**
     * Runs every CommandScheduler loop.  Implements a small state machine:
     * <ol>
     *   <li><b>IDLE</b> — poll sensor; if ball detected → FLIPPING.</li>
     *   <li><b>FLIPPING</b> — hold flip position for {@link #FLIP_HOLD_MS}
     *       then → RETURNING.</li>
     *   <li><b>RETURNING</b> — return to rest; wait for cooldown then → IDLE.</li>
     * </ol>
     */
    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                if (isBallDetected()) {
                    flipperServo.setPosition(FLIP_POSITION);
                    stateTimer.reset();
                    state = FlipperState.FLIPPING;
                }
                break;

            case FLIPPING:
                if (stateTimer.milliseconds() >= FLIP_HOLD_MS) {
                    flipperServo.setPosition(REST_POSITION);
                    stateTimer.reset();
                    state = FlipperState.RETURNING;
                }
                break;

            case RETURNING:
                if (stateTimer.milliseconds() >= COOLDOWN_MS) {
                    state = FlipperState.IDLE;
                }
                break;
        }

        // Telemetry
        telemetry.addData("Flipper State", state);
        telemetry.addData("Flipper Dist (cm)", "%.1f", getDistanceCm());
    }
}
