package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Designed for the 2026-2027 BIOBUZZ game pieces (Pollen & Nectar)
 * using a passive spring-loaded backboard.
 */
public class TurretShooterSubsystem extends SubsystemBase {

    // ── Constants ─────────────────────────────────────────────────────────────
    public static final double MOTOR_MAX_RPM         = 6000.0;
    public static final double ENCODER_TICKS_PER_REV = 28.0;
    public static final double RPM_TOLERANCE         = 50.0;

    // ── PID Coefficients ──────────────────────────────────────────────────────
    public static double kP   = 0.005;
    public static double kI   = 0.01;
    public static double kD   = 0.00005;
    public static double maxI = 0.3; // Integral anti-windup limit

    // ── Feedforward Coefficients ──────────────────────────────────────────────
    public static double kS = 8.07;    // Static friction / base offset
    public static double kV = 0.00067; // Velocity constant
    public static double kA = 0.0;     // Acceleration constant

    // ── Shooting Speed Values (Ticks per second) ─────────────────────────────
    public static double outtakeVelocityShort = 925.0;  // ~1982 RPM
    public static double outtakeVelocityLong  = 1075.0; // ~2304 RPM
    public static double maxAccel             = 30000.0;

    // ── Experimental Distance Scaling ────────────────────────────────────────
    public static double shotSpeedSlope     = 5.40541;
    public static double shotSpeedIntercept = 675.67568;
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

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     */
    public TurretShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "shooterMotor");
    }

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     * @param motorName   primary hardware configuration name for the shooter motor
     */
    public TurretShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String motorName) {
        this.telemetry = telemetry;

        shooterMotor = initShooterMotor(hardwareMap, motorName);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastTimeNanos = System.nanoTime();
    }

    private static DcMotorEx initShooterMotor(HardwareMap hardwareMap, String motorName) {
        try {
            return hardwareMap.get(DcMotorEx.class, motorName);
        } catch (Exception e) {
            try {
                return hardwareMap.get(DcMotorEx.class, "turretShooterLeft");
            } catch (Exception e2) {
                return hardwareMap.get(DcMotorEx.class, "turretShooter");
            }
        }
    }

    // ───────────────────────────── Speed Control ──────────────────────────────

    /**
     * Start spinning the shooter at a given target velocity (ticks/sec).
     *
     * @param ticksPerSecond target velocity in encoder ticks per second
     */
    public void setTargetVelocity(double ticksPerSecond) {
        this.targetVelocity = Math.max(shotSpeedMin, Math.min(shotSpeedMax, ticksPerSecond));
        this.isSpinning = (targetVelocity > 0);
        if (!isSpinning) {
            integralSum = 0.0;
            lastError = 0.0;
            state = ShooterState.IDLE;
        }
    }

    /**
     * Start spinning the shooter at a given target RPM.
     *
     * @param targetRPM target revolutions per minute
     */
    public void setTargetRPM(double targetRPM) {
        double ticksPerSecond = (targetRPM / 60.0) * ENCODER_TICKS_PER_REV;
        setTargetVelocity(ticksPerSecond);
    }

    /**
     * Spin up for short-range shot (~925 ticks/sec / ~1982 RPM).
     */
    public void spinUpShort() {
        setTargetVelocity(outtakeVelocityShort);
    }

    /**
     * Spin up for long-range shot (~1075 ticks/sec / ~2304 RPM).
     */
    public void spinUpLong() {
        setTargetVelocity(outtakeVelocityLong);
    }

    /**
     * Spin up proportionally based on trigger input (0.0 to 1.0).
     */
    public void spinWithTrigger(double triggerVal) {
        if (triggerVal < 0.05) {
            return;
        }
        double target = outtakeVelocityShort + triggerVal * (outtakeVelocityLong - outtakeVelocityShort);
        setTargetVelocity(target);
    }

    /**
     * Calculate target speed from distance using experimental linear scaling:
     * speed = slope * distance + intercept
     */
    public double calculateSpeedForDistance(double distanceInches) {
        double calcSpeed = (shotSpeedSlope * distanceInches) + shotSpeedIntercept;
        return Math.max(shotSpeedMin, Math.min(shotSpeedMax, calcSpeed));
    }

    /**
     * Stop the turret shooter motor immediately and reset PID state.
     */
    public void stop() {
        this.targetVelocity = 0.0;
        this.isSpinning = false;
        this.integralSum = 0.0;
        this.lastError = 0.0;
        this.state = ShooterState.IDLE;
        shooterMotor.setPower(0.0);
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
        return shooterMotor.getCurrentPosition();
    }

    public double getAngularVelocityDegPerSec() {
        return shooterMotor.getVelocity(AngleUnit.DEGREES);
    }

    public double getMotorPower() {
        return shooterMotor.getPower();
    }

    public ShooterState getState() {
        return state;
    }

    public boolean isAtTargetSpeed() {
        return isSpinning && Math.abs(getTargetRPM() - getCurrentRPM()) <= RPM_TOLERANCE;
    }

    public DcMotorEx getShooterMotor() {
        return shooterMotor;
    }

    // ───────────────────────────── Background Automation ──────────────────────

    @Override
    public void periodic() {
        long nowNanos = System.nanoTime();
        double rawDt = (nowNanos - lastTimeNanos) / 1e9;
        double dt = (rawDt <= 0 || rawDt > 0.2) ? 0.02 : rawDt;
        lastTimeNanos = nowNanos;

        // Read raw motor encoder velocity (ticks/second)
        currentVelocity = shooterMotor.getVelocity();

        double currentRPM = getCurrentRPM();
        double targetRPM  = getTargetRPM();
        double rpmError   = targetRPM - currentRPM;

        if (!isSpinning || targetVelocity <= 0) {
            shooterMotor.setPower(0.0);
            integralSum = 0.0;
            lastError = 0.0;
            state = ShooterState.IDLE;
        } else {
            // Update shooter state
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

            // PID output
            double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);

            // Feedforward calculation
            double feedForward = (kS * Math.signum(targetVelocity)) + (kV * targetVelocity);

            // Total motor power (-1.0 to 1.0)
            double unclampedPower = pidOutput + feedForward;
            double power = Math.max(-1.0, Math.min(1.0, unclampedPower));

            shooterMotor.setPower(power);
        }

        // Detailed Telemetry Output
        if (telemetry != null) {
            telemetry.addData("Shooter State", state);
            telemetry.addData("At Target Speed", isAtTargetSpeed());
            telemetry.addData("Current RPM", "%.1f", currentRPM);
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("PID Error (RPM)", "%.1f", rpmError);
            telemetry.addData("Motor Power", "%.3f", shooterMotor.getPower());
            telemetry.addData("Encoder Velocity (ticks/s)", "%.1f", currentVelocity);
            telemetry.addData("Encoder Position (ticks)", shooterMotor.getCurrentPosition());
            telemetry.addData("Angular Velocity (deg/s)", "%.1f", getAngularVelocityDegPerSec());
        }
    }
}
