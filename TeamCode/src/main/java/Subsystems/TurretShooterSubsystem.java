package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TurretShooterSubsystem — Controls the dual-motor flywheel shooter mechanism.
 * Designed for the 2026-2027 BIOBUZZ game pieces (Pollen & Nectar).
 *
 * <h3>Hardware Configuration</h3>
 * <ul>
 *   <li>{@code shooterMotor1} — Primary dual-flywheel motor ("shooterMotor1" or fallback "shooterMotor").</li>
 *   <li>{@code shooterMotor2} — Secondary dual-flywheel motor ("shooterMotor2").</li>
 * </ul>
 */
public class TurretShooterSubsystem extends SubsystemBase {

    // ── Constants ─────────────────────────────────────────────────────────────
    public static final double MOTOR_MAX_RPM         = 6000.0;
    public static final double ENCODER_TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE         = 50.0;

    // ── Motor Directions ──────────────────────────────────────────────────────
    public static DcMotorSimple.Direction MOTOR_1_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction MOTOR_2_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // ── PID & Feedforward Coefficients ───────────────────────────────────────
    public static double kP   = 0.0015;
    public static double kI   = 0.001;
    public static double kD   = 0.00001;
    public static double maxI = 0.3; // Integral anti-windup limit

    // ── Rapid Post-Shot Recovery / Bang-Bang Boost ───────────────────────────
    /** Enable full-power acceleration boost when velocity drops after a shot. */
    public static boolean useRecoveryBoost      = true;
    /** RPM drop threshold below target to activate max-power recovery boost. */
    public static double recoveryThresholdRPM   = 25.0;
    /** Motor power (0.0 to 1.0) applied during fast recovery boost. */
    public static double recoveryPower          = 1.0;

    // ── Feedforward Coefficients (normalized 0.0 to 1.0 power) ────────────────
    public static double kS = 0.015;   // Base power to overcome static friction
    public static double kV = 0.000275;// Calibrated feedforward gain for exact 4700 RPM steady-state
    public static double kA = 0.0;     // Acceleration constant

    // ── Shooting Speed Values (Ticks per second) ─────────────────────────────
    public static double outtakeVelocityShort = 2193.33; // 4700 RPM (4700/60 * 28 = 2193.33 ticks/sec)
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
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
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
        this(hardwareMap, telemetry, "shooterMotor1", "shooterMotor2");
    }

    public TurretShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String motor1Name, String motor2Name) {
        this.telemetry = telemetry;

        // Try primary motor1Name (fallback to legacy "shooterMotor" if "shooterMotor1" is absent)
        DcMotorEx m1 = null;
        try {
            m1 = hardwareMap.get(DcMotorEx.class, motor1Name);
        } catch (Exception e) {
            try {
                m1 = hardwareMap.get(DcMotorEx.class, "shooterMotor");
            } catch (Exception ignored) {}
        }
        this.shooterMotor1 = m1;

        // Try motor2Name ("shooterMotor2")
        DcMotorEx m2 = null;
        try {
            m2 = hardwareMap.get(DcMotorEx.class, motor2Name);
        } catch (Exception ignored) {}
        this.shooterMotor2 = m2;

        // Configure Motor 1
        if (shooterMotor1 != null) {
            shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor1.setDirection(MOTOR_1_DIRECTION);
            shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Configure Motor 2
        if (shooterMotor2 != null) {
            shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor2.setDirection(MOTOR_2_DIRECTION);
            shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if (shooterMotor1 != null) shooterMotor1.setPower(0.0);
        if (shooterMotor2 != null) shooterMotor2.setPower(0.0);
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
        if (shooterMotor1 != null) return shooterMotor1.getCurrentPosition();
        if (shooterMotor2 != null) return shooterMotor2.getCurrentPosition();
        return 0;
    }

    public double getAngularVelocityDegPerSec() {
        if (shooterMotor1 != null) return shooterMotor1.getVelocity(AngleUnit.DEGREES);
        if (shooterMotor2 != null) return shooterMotor2.getVelocity(AngleUnit.DEGREES);
        return 0.0;
    }

    public double getMotor1Power() {
        return (shooterMotor1 != null) ? shooterMotor1.getPower() : 0.0;
    }

    public double getMotor2Power() {
        return (shooterMotor2 != null) ? shooterMotor2.getPower() : 0.0;
    }

    public double getMotorPower() {
        return getMotor1Power();
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
        if (shooterMotor1 == null && shooterMotor2 == null) return;

        long nowNanos = System.nanoTime();
        double rawDt = (nowNanos - lastTimeNanos) / 1e9;
        double dt = (rawDt <= 0 || rawDt > 0.2) ? 0.02 : rawDt;
        lastTimeNanos = nowNanos;

        if (!isSpinning || targetVelocity <= 0) {
            if (shooterMotor1 != null) shooterMotor1.setPower(0.0);
            if (shooterMotor2 != null) shooterMotor2.setPower(0.0);
            integralSum = 0.0;
            lastError = 0.0;
            isRecoveryActive = false;
            state = ShooterState.IDLE;
        } else {
            // Read raw motor encoder velocity (ticks/second) from available motor
            try {
                double vel1 = (shooterMotor1 != null) ? shooterMotor1.getVelocity() : Double.NaN;
                double vel2 = (shooterMotor2 != null) ? shooterMotor2.getVelocity() : Double.NaN;
                double vel = !Double.isNaN(vel1) ? vel1 : (!Double.isNaN(vel2) ? vel2 : 0.0);
                currentVelocity = Math.abs(vel);
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
            isRecoveryActive = useRecoveryBoost && (rpmError > recoveryThresholdRPM);

            double power;
            if (isRecoveryActive) {
                power = Math.signum(targetVelocity) * recoveryPower;
                integralSum = 0.0;
                lastError = targetVelocity - currentVelocity;
            } else {
                // Standard PID error calculation (in ticks/sec)
                double error = targetVelocity - currentVelocity;

                integralSum += error * dt;
                if (kI > 0) {
                    double maxIntegralTerm = maxI / kI;
                    integralSum = Math.max(-maxIntegralTerm, Math.min(maxIntegralTerm, integralSum));
                }

                double derivative = (error - lastError) / dt;
                lastError = error;

                double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);
                double feedForward = (kS * Math.signum(targetVelocity)) + (kV * targetVelocity);

                double unclampedPower = pidOutput + feedForward;
                power = Math.max(-1.0, Math.min(1.0, unclampedPower));

                if (Double.isNaN(power) || (power <= 0.05 && targetVelocity > 0)) {
                    power = 0.5;
                }
            }

            // Command power to both dual flywheel motors
            if (shooterMotor1 != null) shooterMotor1.setPower(power);
            if (shooterMotor2 != null) shooterMotor2.setPower(power);
        }

        if (telemetry != null) {
            telemetry.addData("Shooter Motor 1 Found", (shooterMotor1 != null));
            telemetry.addData("Shooter Motor 2 Found", (shooterMotor2 != null));
            telemetry.addData("Shooter Spinning Flag", isSpinning);
            telemetry.addData("Shooter State", state);
            telemetry.addData("Shooter Recovery Active", isRecoveryActive);
            telemetry.addData("Shooter Motor 1 Power", (shooterMotor1 != null) ? shooterMotor1.getPower() : 0.0);
            telemetry.addData("Shooter Motor 2 Power", (shooterMotor2 != null) ? shooterMotor2.getPower() : 0.0);
            telemetry.addData("Shooter Target RPM", getTargetRPM());
            telemetry.addData("Shooter Current RPM", getCurrentRPM());
        }
    }
}
