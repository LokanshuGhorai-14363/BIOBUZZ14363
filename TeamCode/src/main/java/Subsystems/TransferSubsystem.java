package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TransferSubsystem — controls transfer motor with timer-based active sequence and cooldown.
 *
 * <h3>Hardware</h3>
 * <ul>
 *   <li>{@code transferMotor} — goBILDA Yellow Jacket 1150 RPM motor ("transferMotor").</li>
 * </ul>
 *
 * <h3>Behavior</h3>
 * When triggered manually (e.g. via gamepad button/trigger), the transfer motor runs at full power (1.0) for
 * {@link #TRANSFER_RUN_MS} milliseconds to move the game element from intake to the scoring mechanism.
 * A cooldown timer ({@link #COOLDOWN_MS}) prevents rapid re-triggering.
 */
public class TransferSubsystem extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotorEx transferMotor;
    private final Telemetry telemetry;

    // ── Constants ─────────────────────────────────────────────────────────────
    /** Full motor power used for transferring game elements (1.0). */
    public static final double FULL_POWER      = 1.0;
    /** Time in ms to run the transfer motor during a transfer sequence. */
    public static final long   TRANSFER_RUN_MS = 500;
    /** Cooldown in ms after a transfer before another can trigger. */
    public static final long   COOLDOWN_MS     = 1000;

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
        this(hardwareMap, telemetry, "transferMotor");
    }

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     * @param motorName   hardware configuration name for the transfer motor
     */
    public TransferSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String motorName) {
        this.telemetry = telemetry;

        transferMotor = hardwareMap.get(DcMotorEx.class, motorName);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setPower(0.0);
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

    @Override
    public void periodic() {
        switch (state) {
            case IDLE:
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
        }
    }
}
