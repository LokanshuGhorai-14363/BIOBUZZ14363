package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * TransferSubsystem — automated game-element detection and transfer into the scoring mechanism (turret shooter).
 *
 * <h3>Hardware</h3>
 * <ul>
 *   <li>{@code transferMotor}  — goBILDA Yellow Jacket 1150 RPM motor ("transferMotor").</li>
 *   <li>{@code transferSensor} — REV 2m Distance Sensor ("transferSensor").</li>
 * </ul>
 *
 * <h3>Behavior</h3>
 * When triggered manually (e.g. via gamepad button/trigger) or when the distance sensor reads
 * &lt; {@link #DETECT_THRESHOLD_CM}, the transfer motor runs at full power (1.0) for
 * {@link #TRANSFER_RUN_MS} milliseconds to move the game element from intake to the scoring mechanism.
 * A cooldown timer ({@link #COOLDOWN_MS}) prevents rapid re-triggering.
 * This logic runs inside {@link #periodic()} so it behaves as an automated background task
 * while also supporting manual trigger invocations.
 */
public class TransferSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotorEx      transferMotor;
    private final DistanceSensor transferSensor;
    private final Telemetry      telemetry;

    // ── Constants ─────────────────────────────────────────────────────────────
    /** Distance threshold in cm that indicates a game element is present. */
    public static final double DETECT_THRESHOLD_CM = 5.0;
    /** Full motor power used for transferring game elements (1.0). */
    public static final double FULL_POWER          = 1.0;
    /** Time in ms to run the transfer motor during a transfer sequence. */
    public static final long   TRANSFER_RUN_MS     = 500;
    /** Cooldown in ms after a transfer before another can trigger. */
    public static final long   COOLDOWN_MS         = 1000;

    // ── State machine ─────────────────────────────────────────────────────────
    public enum TransferState { IDLE, TRANSFERRING, COOLDOWN }
    private TransferState state = TransferState.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // ── Constructors ──────────────────────────────────────────────────────────

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     */
    public TransferSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "transferMotor", "transferSensor");
    }

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     * @param motorName   hardware configuration name for the transfer motor
     * @param sensorName  hardware configuration name for the distance sensor
     */
    public TransferSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String motorName, String sensorName) {
        this.telemetry = telemetry;

        transferMotor = hardwareMap.get(DcMotorEx.class, motorName);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setPower(0.0);

        transferSensor = initSensor(hardwareMap, telemetry, sensorName);
    }

    private static DistanceSensor initSensor(HardwareMap hardwareMap, Telemetry telemetry, String sensorName) {
        try {
            return hardwareMap.get(DistanceSensor.class, sensorName);
        } catch (Exception e) {
            if (telemetry != null) {
                telemetry.addData("Warning", "Transfer sensor missing: " + sensorName);
            }
            return null;
        }
    }

    // ───────────────────────────── Actions & Controls ─────────────────────────

    /**
     * Manually triggers a transfer sequence if the subsystem is currently IDLE.
     * Runs the transfer motor at full power (1.0) to move the game element to the scoring mechanism.
     */
    public void transfer() {
        if (state == TransferState.IDLE) {
            startTransfer();
        }
    }

    /**
     * Internal helper to initiate motor spin and state transition to TRANSFERRING.
     */
    private void startTransfer() {
        transferMotor.setPower(FULL_POWER);
        stateTimer.reset();
        state = TransferState.TRANSFERRING;
    }

    /**
     * Stops the transfer motor immediately and resets power to 0.0.
     */
    public void stop() {
        transferMotor.setPower(0.0);
        state = TransferState.IDLE;
    }

    /**
     * @return current distance reading in centimeters, or Double.MAX_VALUE if sensor is unavailable.
     */
    public double getDistanceCm() {
        if (transferSensor == null) {
            return Double.MAX_VALUE;
        }
        return transferSensor.getDistance(DistanceUnit.CM);
    }

    /**
     * @return {@code true} if a game element is detected within threshold distance.
     */
    public boolean isElementDetected() {
        return getDistanceCm() < DETECT_THRESHOLD_CM;
    }

    /**
     * @return current motor power level (-1.0 to 1.0).
     */
    public double getMotorPower() {
        return transferMotor.getPower();
    }

    /**
     * @return current {@link TransferState}.
     */
    public TransferState getState() {
        return state;
    }

    /**
     * @return underlying {@link DcMotorEx} instance.
     */
    public DcMotorEx getTransferMotor() {
        return transferMotor;
    }

    // ───────────────────────────── Background automation ──────────────────────

    /**
     * Runs every CommandScheduler loop. Implements automated state machine:
     * <ol>
     *   <li><b>IDLE</b> — poll sensor; if element detected → TRANSFERRING.</li>
     *   <li><b>TRANSFERRING</b> — run motor for {@link #TRANSFER_RUN_MS} ms then → COOLDOWN.</li>
     *   <li><b>COOLDOWN</b> — stop motor; wait for {@link #COOLDOWN_MS} ms then → IDLE.</li>
     * </ol>
     */
    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
                if (isElementDetected()) {
                    startTransfer();
                }
                break;

            case TRANSFERRING:
                if (stateTimer.milliseconds() >= TRANSFER_RUN_MS) {
                    transferMotor.setPower(0.0);
                    stateTimer.reset();
                    state = TransferState.COOLDOWN;
                }
                break;

            case COOLDOWN:
                if (stateTimer.milliseconds() >= COOLDOWN_MS) {
                    state = TransferState.IDLE;
                }
                break;
        }

        // Telemetry
        if (telemetry != null) {
            telemetry.addData("Transfer State", state);
            telemetry.addData("Transfer Power", transferMotor.getPower());
            telemetry.addData("Transfer Dist (cm)", "%.1f", getDistanceCm());
        }
    }
}
