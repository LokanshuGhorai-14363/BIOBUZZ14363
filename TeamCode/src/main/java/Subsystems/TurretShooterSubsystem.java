package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TurretShooterSubsystem — Controls the flywheel shooter motor.
 * Designed for the 2026-2027 BIOBUZZ game pieces (Pollen & Nectar).
 */
public class TurretShooterSubsystem extends SubsystemBase {

    // ── Constants ─────────────────────────────────────────────────────────────
    public static final double MOTOR_MAX_RPM         = 6000.0;
    public static final double ENCODER_TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE         = 50.0;

    // ── PID & Feedforward Coefficients ───────────────────────────────────────
    public static double kP         = 0.0035;
    public static double kI         = 0.0005;
    public static double kD         = 0.00005;
    public static double maxI       = 0.15; // Strict anti-windup limit
    public static double iZoneRPM   = 20.0; // Only accumulate integral within ±20 RPM of target

    // ── Rapid Post-Shot Recovery / Bang-Bang Boost ───────────────────────────
    /** Enable full-power acceleration boost when velocity drops after a shot. */
    public static boolean useRecoveryBoost      = true;
    /** RPM drop threshold below target to activate max-power recovery boost. */
    public static double recoveryThresholdRPM   = 25.0;
    /** Motor power (0.0 to 1.0) applied during fast recovery boost. */
    public static double recoveryPower          = 1.0;

    // ── Feedforward Coefficients (normalized 0.0 to 1.0 power) ────────────────
    public static double kS = 0.015;   // Base power to overcome static friction
    public static double kV = 0.000275;// Calibrated feedforward gain for exact 4699 RPM steady-state
    public static double kA = 0.0;     // Acceleration constant

    // ── Shooting Speed Values (Ticks per second) ─────────────────────────────
    public static double outtakeVelocityShort = 2192.87; // 4699 RPM (4699/60 * 28 = 2192.87 ticks/sec)
    public static double outtakeVelocityLong  = 2800.0;  // ~6000 RPM

    public static double shotSpeedMin       = 0.0;
    public static double shotSpeedMax       = 3500.0; // ~7500 RPM max ticks/sec

    // ── Shooter State ─────────────────────────────────────────────────────────
    public enum ShooterState {
        IDLE,
        SPINNING_UP,
        AT_SPEED
    }

    // ── Hardware & State ──────────────────────────────────────────────────────
    private final DcMotorEx shooterMotor;
    private final Telemetry telemetry;

    private ShooterState state = ShooterState.IDLE;
    private double targetVelocity = 0.0;  // Ticks per second
    private double currentVelocity = 0.0; // Ticks per second
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTimeNanos = System.nanoTime();
    private boolean isSpinning = true;
    private boolean isRecoveryActive = false;

    public TurretShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "shooterMotor");
    }

    public TurretShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String motorName) {
        this.telemetry = telemetry;

        shooterMotor = hardwareMap.get(DcMotorEx.class, motorName);
        if (shooterMotor != null) {
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        lastTimeNanos = System.nanoTime();
    }

    // ───────────────────────────── Speed Control ──────────────────────────────

    public void setTargetVelocity(double ticksPerSecond) {
        this.targetVelocity = Math.max(shotSpeedMin, Math.min(shotSpeedMax, ticksPerSecond));
        this.isSpinning = (targetVelocity > 0);
        if (!isSpinning) {
            integralSum = 0.0;
            lastError = 0.0;
            state = ShooterState.IDLE;
        }
    }

    public void setTargetRPM(double targetRPM) {
        double ticksPerSecond = (targetRPM / 60.0) * ENCODER_TICKS_PER_REV;
        setTargetVelocity(ticksPerSecond);
    }

    public void spinUpShort() {
        setTargetVelocity(outtakeVelocityShort);
    }

    public void spinUpLong() {
        setTargetVelocity(outtakeVelocityLong);
    }

    /**
     * Spin up proportionally based on trigger input (0.0 to 1.0).
     */
    public void spinWithTrigger(double triggerVal) {
        if (triggerVal < 0.05) {
            stop();
            return;
        }
        // Scale trigger input (0.05..1.0) to target velocity
        double target = outtakeVelocityShort + (triggerVal * (outtakeVelocityLong - outtakeVelocityShort));
        setTargetVelocity(target);
    }

    public void stop() {
        this.targetVelocity = 0.0;
        this.isSpinning = false;
        this.integralSum = 0.0;
        this.lastError = 0.0;
        this.isRecoveryActive = false;
        this.state = ShooterState.IDLE;
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
    }

    // ───────────────────────────── Accessors & Encoder Readers ────────────────

    public boolean isSpinning() {
        return isSpinning;
    }

    public boolean isRecoveryActive() {
        return isRecoveryActive;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetRPM() {
        return (targetVelocity / ENCODER_TICKS_PER_REV) * 60.0;
    }

    public double getCurrentRPM() {
        return (currentVelocity / ENCODER_TICKS_PER_REV) * 60.0;
    }

    public double getVelocityTicksPerSec() {
        return currentVelocity;
    }

    public int getRawEncoderPosition() {
        return (shooterMotor != null) ? shooterMotor.getCurrentPosition() : 0;
    }

    public double getAngularVelocityDegPerSec() {
        return (shooterMotor != null) ? shooterMotor.getVelocity(AngleUnit.DEGREES) : 0.0;
    }

    public double getMotorPower() {
        return (shooterMotor != null) ? shooterMotor.getPower() : 0.0;
    }

    public ShooterState getState() {
        return state;
    }

    public boolean isAtTargetSpeed() {
        return isSpinning && Math.abs(getTargetRPM() - getCurrentRPM()) <= RPM_TOLERANCE;
    }

    // ───────────────────────────── Background Automation ──────────────────────

    @Override
    public void periodic() {
        if (shooterMotor == null) return;

        long nowNanos = System.nanoTime();
        double rawDt = (nowNanos - lastTimeNanos) / 1e9;
        double dt = (rawDt <= 0 || rawDt > 0.2) ? 0.02 : rawDt;
        lastTimeNanos = nowNanos;

        if (!isSpinning || targetVelocity <= 0) {
            shooterMotor.setPower(0.0);
            integralSum = 0.0;
            lastError = 0.0;
            isRecoveryActive = false;
            state = ShooterState.IDLE;
        } else {
            // Read raw motor encoder velocity (ticks/second)
            try {
                double vel = shooterMotor.getVelocity();
                currentVelocity = Double.isNaN(vel) ? 0.0 : Math.abs(vel);
            } catch (Exception e) {
                currentVelocity = 0.0;
            }

            double currentRPM = getCurrentRPM();
            double targetRPM  = getTargetRPM();
            double rpmError   = targetRPM - currentRPM;

            if (Math.abs(rpmError) <= RPM_TOLERANCE) {
                state = ShooterState.AT_SPEED;
            } else {
                state = ShooterState.SPINNING_UP;
            }

            // Rapid Post-Shot Recovery / Bang-Bang Boost Check:
            // When current RPM drops significantly below target RPM (e.g. immediately after a shot),
            // apply maximum motor power (1.0) to achieve the fastest possible physical recovery.
            isRecoveryActive = useRecoveryBoost && (rpmError > recoveryThresholdRPM);

            double power;
            if (isRecoveryActive) {
                power = Math.signum(targetVelocity) * recoveryPower;
                integralSum = 0.0; // Reset integral accumulation during maximum boost phase
                lastError = targetVelocity - currentVelocity;
            } else {
                // Standard PID error calculation (in ticks/sec)
                double error = targetVelocity - currentVelocity;

                // Anti-Windup for Rapid Fire:
                // 1. Only accumulate integral within iZone (±20 RPM of target)
                // 2. Dump integral sum whenever current speed reaches or exceeds target RPM
                if (currentRPM >= targetRPM || Math.abs(rpmError) > iZoneRPM) {
                    integralSum = 0.0;
                } else {
                    integralSum += error * dt;
                    if (kI > 0) {
                        double maxIntegralTerm = maxI / kI;
                        integralSum = Math.max(-maxIntegralTerm, Math.min(maxIntegralTerm, integralSum));
                    }
                }

                // Derivative calculation
                double derivative = (error - lastError) / dt;
                lastError = error;

                double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);
                double feedForward = (kS * Math.signum(targetVelocity)) + (kV * targetVelocity);

                double unclampedPower = pidOutput + feedForward;
                power = Math.max(-1.0, Math.min(1.0, unclampedPower));

                // Fallback: Ensure motor receives power if calculation yields NaN
                if (Double.isNaN(power)) {
                    power = 0.5;
                }
            }

            shooterMotor.setPower(power);
        }

        if (telemetry != null) {
            telemetry.addData("Shooter Motor Found", (shooterMotor != null));
            telemetry.addData("Shooter Spinning Flag", isSpinning);
            telemetry.addData("Shooter State", state);
            telemetry.addData("Shooter Recovery Active", isRecoveryActive);
            telemetry.addData("Shooter Power", (shooterMotor != null) ? shooterMotor.getPower() : 0.0);
            telemetry.addData("Shooter Target RPM", getTargetRPM());
            telemetry.addData("Shooter Current RPM", getCurrentRPM());
        }
    }
}
