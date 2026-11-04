package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * TurretShooterSubsystem — Controls the flywheel shooter motor.
 * Designed for the 2026-2027 BIOBUZZ game pieces (Pollen & Nectar).
 */
public class TurretShooterSubsystem extends SubsystemBase {

    // ── Constants ─────────────────────────────────────────────────────────────
    public static final double MOTOR_MAX_RPM         = 6000.0;
    public static final double ENCODER_TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE         = 50.0;

    // ── PID Coefficients ──────────────────────────────────────────────────────
    public static double kP   = 0.001;
    public static double kI   = 0.001;
    public static double kD   = 0.00001;
    public static double maxI = 0.3; // Integral anti-windup limit

    // ── Feedforward Coefficients (normalized 0.0 to 1.0 power) ────────────────
    public static double kS = 0.05;    // Base power to overcome static friction
    public static double kV = 0.00035; // Velocity feedforward gain (1.0 / ~2800 ticks/sec)
    public static double kA = 0.0;     // Acceleration constant

    // ── Shooting Speed Values (Ticks per second) ─────────────────────────────
    public static double outtakeVelocityShort = 925.0;  // ~1982 RPM
    public static double outtakeVelocityLong  = 1075.0; // ~2304 RPM

    public static double shotSpeedMin       = 0.0;
    public static double shotSpeedMax       = 2800.0; // ~6000 RPM max ticks/sec

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
    private boolean isSpinning = false;

    public TurretShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "shooterMotor");
    }

    public TurretShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String motorName) {
        this.telemetry = telemetry;

        shooterMotor = initShooterMotor(hardwareMap, motorName);
        if (shooterMotor != null) {
            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        lastTimeNanos = System.nanoTime();
    }

    private static DcMotorEx initShooterMotor(HardwareMap hardwareMap, String motorName) {
        String[] possibleNames = {
            motorName, "shooterMotor", "shooter", "flywheel", 
            "turretShooter", "turretShooterLeft", "shooter_motor", "flywheelMotor", "outtake"
        };
        for (String name : possibleNames) {
            if (name == null || name.isEmpty()) continue;
            try {
                DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
                if (motor != null) return motor;
            } catch (Exception ignored) {}
        }
        try {
            List<DcMotorEx> motors = hardwareMap.getAll(DcMotorEx.class);
            if (!motors.isEmpty()) return motors.get(0);
        } catch (Exception ignored) {}
        return null;
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
        this.state = ShooterState.IDLE;
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
    }

    // ───────────────────────────── Accessors & Encoder Readers ────────────────

    public boolean isSpinning() {
        return isSpinning;
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
            state = ShooterState.IDLE;
        } else {
            // Read raw motor encoder velocity (ticks/second)
            try {
                double vel = shooterMotor.getVelocity();
                currentVelocity = Double.isNaN(vel) ? 0.0 : vel;
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

            // PID error calculation (in ticks/sec)
            double error = targetVelocity - currentVelocity;

            // Integral accumulation with anti-windup clamping
            integralSum += error * dt;
            if (kI > 0) {
                double maxIntegralTerm = maxI / kI;
                integralSum = Math.max(-maxIntegralTerm, Math.min(maxIntegralTerm, integralSum));
            }

            // Derivative calculation
            double derivative = (error - lastError) / dt;
            lastError = error;

            double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);
            double feedForward = (kS * Math.signum(targetVelocity)) + (kV * targetVelocity);

            double unclampedPower = pidOutput + feedForward;
            double power = Math.max(-1.0, Math.min(1.0, unclampedPower));

            // Fallback: Ensure motor receives power when target is set even if calculation yields low/NaN power
            if (Double.isNaN(power) || (power <= 0.05 && targetVelocity > 0)) {
                power = 0.5;
            }

            shooterMotor.setPower(power);
        }

        if (telemetry != null) {
            telemetry.addData("Shooter Motor Found", (shooterMotor != null));
            telemetry.addData("Shooter Spinning Flag", isSpinning);
            telemetry.addData("Shooter State", state);
            telemetry.addData("Shooter Power", (shooterMotor != null) ? shooterMotor.getPower() : 0.0);
            telemetry.addData("Shooter Target RPM", getTargetRPM());
            telemetry.addData("Shooter Current RPM", getCurrentRPM());
        }
    }
}
