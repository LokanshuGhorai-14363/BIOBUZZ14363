package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/**
 * TurretShooterSubsystem — Controls the dual-motor flywheel shooter mechanism.
 * Designed for the 2026-2027 BIOBUZZ game pieces (Pollen & Nectar).
 *
 * <h3>Hardware Configuration</h3>
 * <ul>
 *   <li>{@code shooterMotor1} — Primary flywheel motor ("shooterMotor1" or fallback "shooterMotor").</li>
 *   <li>{@code shooterMotor2} — Secondary flywheel motor ("shooterMotor2") on opposite end of shaft.</li>
 * </ul>
 */
public class TurretShooterSubsystem extends SubsystemBase {

    // ── Constants ─────────────────────────────────────────────────────────────
    public static final double MOTOR_MAX_RPM         = 6000.0;
    public static final double ENCODER_TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE         = 50.0;

    // ── Motor Directions ──────────────────────────────────────────────────────
    // Motor 2 is physically mounted on the opposite end of the shaft, so it MUST be REVERSED in software.
    public static DcMotorSimple.Direction MOTOR_1_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction MOTOR_2_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // ── PID & Feedforward Coefficients ───────────────────────────────────────
    public static double kP   = 0.0004;
    public static double kI   = 0.00005;
    public static double kD   = 0.00002;
    public static double maxI = 0.05; // Integral anti-windup limit

    // ── Rapid Post-Shot Recovery / Bang-Bang Boost ───────────────────────────
    /** Enable full-power acceleration boost when velocity drops significantly after a shot. */
    public static boolean useRecoveryBoost      = false;
    /** RPM drop threshold below target to activate max-power recovery boost (e.g. 200 RPM during ball exit). */
    public static double recoveryThresholdRPM   = 200.0;
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
    private double motor1Velocity = 0.0;  // Ticks per second
    private double motor2Velocity = 0.0;  // Ticks per second
    private double motor1RPM = 0.0;
    private double motor2RPM = 0.0;

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
        if (shooterMotor1 != null && shooterMotor2 != null) {
            return (motor1RPM + motor2RPM) / 2.0;
        } else if (shooterMotor1 != null) {
            return motor1RPM;
        } else if (shooterMotor2 != null) {
            return motor2RPM;
        }
        return (currentVelocity / ENCODER_TICKS_PER_REV) * 60.0;
    }

    public double getMotor1RPM() {
        return motor1RPM;
    }

    public double getMotor2RPM() {
        return motor2RPM;
    }

    public double getRPMDelta() {
        return Math.abs(motor1RPM - motor2RPM);
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
            motor1Velocity = 0.0;
            motor2Velocity = 0.0;
            motor1RPM = 0.0;
            motor2RPM = 0.0;
            currentVelocity = 0.0;
        } else {
            // Read raw motor encoder velocities (ticks/second) from both motors
            if (shooterMotor1 != null) {
                try {
                    motor1Velocity = shooterMotor1.getVelocity();
                    motor1RPM = (motor1Velocity / ENCODER_TICKS_PER_REV) * 60.0;
                } catch (Exception e) {
                    motor1Velocity = 0.0;
                    motor1RPM = 0.0;
                }
            }
            if (shooterMotor2 != null) {
                try {
                    motor2Velocity = shooterMotor2.getVelocity();
                    motor2RPM = (motor2Velocity / ENCODER_TICKS_PER_REV) * 60.0;
                } catch (Exception e) {
                    motor2Velocity = 0.0;
                    motor2RPM = 0.0;
                }
            }

            if (shooterMotor1 != null && shooterMotor2 != null) {
                currentVelocity = (motor1Velocity + motor2Velocity) / 2.0;
            } else if (shooterMotor1 != null) {
                currentVelocity = motor1Velocity;
            } else {
                currentVelocity = motor2Velocity;
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
            telemetry.addData("Shooter M1 Found", (shooterMotor1 != null));
            telemetry.addData("Shooter M2 Found", (shooterMotor2 != null));
            telemetry.addData("Shooter State", state);
            telemetry.addData("Shooter Target RPM", getTargetRPM());
            telemetry.addData("Shooter M1 RPM", String.format(Locale.US, "%.0f RPM", motor1RPM));
            telemetry.addData("Shooter M2 RPM", String.format(Locale.US, "%.0f RPM", motor2RPM));
            telemetry.addData("Shooter M1/M2 Delta", String.format(Locale.US, "%.0f RPM", getRPMDelta()));
            if (shooterMotor1 != null && shooterMotor2 != null) {
                if (motor1RPM * motor2RPM < -100) {
                    telemetry.addLine("⚠️ WARNING: MOTORS FIGHTING! (Opposite rotation directions)");
                } else if (getRPMDelta() > 300 && isSpinning) {
                    telemetry.addLine("⚠️ WARNING: HIGH M1/M2 RPM DELTA! (Check shaft coupling)");
                }
            }
        }
    }
}
