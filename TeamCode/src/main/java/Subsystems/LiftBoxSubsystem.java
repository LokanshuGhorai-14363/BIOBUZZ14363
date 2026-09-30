package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * LiftBoxSubsystem — dual-CRServo slider/pulley lift + box door/pivot.
 *
 * <h3>Hardware</h3>
 * <ul>
 *   <li>{@code liftLeft}, {@code liftRight} — Axon MAX Mark 2 CRServos
 *       (multi-rotation; treated as continuous-rotation servos).</li>
 *   <li>{@code boxDoor}  — standard servo controlling the cargo door.</li>
 *   <li>{@code boxPivot} — standard servo controlling the dump pivot.</li>
 * </ul>
 *
 * <h3>Axon MAX Mark 2 Timing</h3>
 * At max power the Axon MAX Mark 2 completes roughly 1 revolution per second
 * (60 RPM @ max).  20 rotations ≈ 20 seconds.  A timer tracks elapsed lift
 * time so commands can check {@link #isLiftComplete()}.
 */
public class LiftBoxSubsystem extends SubsystemBase {

    // ── Hardware handles ──────────────────────────────────────────────────────
    private final CRServo liftLeft;
    private final CRServo liftRight;
    private final Servo   boxDoor;
    private final Servo   boxPivot;
    private final Telemetry telemetry;

    // ── Constants ─────────────────────────────────────────────────────────────
    /** Number of full rotations the lift must travel. */
    public static final int    LIFT_ROTATIONS = 20;
    /** Approximate seconds per full revolution at max CRServo power (Axon MAX Mark 2). */
    public static final double SECS_PER_REV   = 1.0;
    /** Total time (seconds) the CRServos must spin to cover LIFT_ROTATIONS. */
    public static final double LIFT_TIME_SEC  = LIFT_ROTATIONS * SECS_PER_REV;

    // Servo positions (0–1 scale, mapped from degrees)
    /** Box door closed / secured position (≈ 50°). */
    public static final double DOOR_CLOSED  = 50.0 / 180.0;   // ≈ 0.278
    /** Box door open / default position (0°). */
    public static final double DOOR_OPEN    = 0.0;
    /** Box pivot dump position (≈ 90°). */
    public static final double PIVOT_DUMP   = 90.0 / 180.0;   // 0.50
    /** Box pivot rest / home position (0°). */
    public static final double PIVOT_HOME   = 0.0;

    // ── State tracking ────────────────────────────────────────────────────────
    private final ElapsedTime liftTimer = new ElapsedTime();
    private boolean liftRunning = false;

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     */
    public LiftBoxSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        liftLeft  = hardwareMap.get(CRServo.class, "liftLeft");
        liftRight = hardwareMap.get(CRServo.class, "liftRight");
        boxDoor   = hardwareMap.get(Servo.class, "boxDoor");
        boxPivot  = hardwareMap.get(Servo.class, "boxPivot");

        // Right servo runs opposite to left for pulley synchronization
        liftRight.setDirection(CRServo.Direction.REVERSE);

        // Initialize to default state: door open, pivot home
        boxDoor.setPosition(DOOR_OPEN);
        boxPivot.setPosition(PIVOT_HOME);
    }

    // ───────────────────────────── Lift control ───────────────────────────────

    /**
     * Begin raising the lift at full CRServo power and start the timer.
     */
    public void startLiftUp() {
        liftTimer.reset();
        liftRunning = true;
        liftLeft.setPower(1.0);
        liftRight.setPower(1.0);
    }

    /**
     * Stop the lift CRServos.
     */
    public void stopLift() {
        liftLeft.setPower(0);
        liftRight.setPower(0);
        liftRunning = false;
    }

    /**
     * Lower the lift back to the home position at full speed (reverse).
     */
    public void startLiftDown() {
        liftTimer.reset();
        liftRunning = true;
        liftLeft.setPower(-1.0);
        liftRight.setPower(-1.0);
    }

    /**
     * @return {@code true} once the lift has been running for the full
     *         {@link #LIFT_TIME_SEC} duration.
     */
    public boolean isLiftComplete() {
        return liftRunning && liftTimer.seconds() >= LIFT_TIME_SEC;
    }

    /**
     * @return {@code true} if lift CRServos are actively spinning.
     */
    public boolean isLiftRunning() {
        return liftRunning;
    }

    // ───────────────────────────── Box control ────────────────────────────────

    /** Close the box door to secure cargo (50°). */
    public void closeDoor() {
        boxDoor.setPosition(DOOR_CLOSED);
    }

    /** Open the box door (0°). */
    public void openDoor() {
        boxDoor.setPosition(DOOR_OPEN);
    }

    /** Rotate the box pivot to dump position (90°). */
    public void pivotDump() {
        boxPivot.setPosition(PIVOT_DUMP);
    }

    /** Return the box pivot to home (0°). */
    public void pivotHome() {
        boxPivot.setPosition(PIVOT_HOME);
    }

    // ───────────────────────────── Full reset ─────────────────────────────────

    /**
     * Reset every actuator to the default/home state:
     * door open, pivot home, lift stopped.
     */
    public void resetToDefault() {
        stopLift();
        openDoor();
        pivotHome();
    }

    // ───────────────────────────── Lifecycle ──────────────────────────────────

    @Override
    public void periodic() {
        telemetry.addData("Lift Running", liftRunning);
        if (liftRunning) {
            telemetry.addData("Lift Time (s)", "%.1f / %.1f",
                    liftTimer.seconds(), LIFT_TIME_SEC);
        }
        telemetry.addData("Door Pos", boxDoor.getPosition());
        telemetry.addData("Pivot Pos", boxPivot.getPosition());
    }
}
