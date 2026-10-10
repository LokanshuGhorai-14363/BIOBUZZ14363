package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TurretShooterSubsystem — Dual goBILDA Yellow Jacket 6000 RPM motors
 * with closed-loop PID + Feedforward velocity control.
 *
 * Designed for the 2026-2027 BIOBUZZ game pieces (Pollen & Nectar)
 * using a passive spring-loaded backboard.
 */
public class TurretShooterSubsystem extends SubsystemBase {

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;

    // ── PID Coefficients ──────────────────────────────────────────────────────
    public static double kP   = 0.005;
    public static double kI   = 0.01;
    public static double kD   = 0.00005;
    public static double maxI = 0.3; // Integral anti-windup limit

    // ── Feedforward Coefficients ──────────────────────────────────────────────
    public static double kS = 0.07;    // Static friction
    public static double kV = 0.00067; // Velocity constant
    public static double kA = 0.0;     // Acceleration constant

    // ── Shooting Speed Values (Ticks per second) ─────────────────────────────
    public static double outtakeVelocityShort = 925.0;
    public static double outtakeVelocityLong  = 1075.0;
    public static double maxAccel             = 30000.0; // Max acceleration limit

    // ── Experimental Distance Scaling ────────────────────────────────────────
    public static double shotSpeedSlope     = 5.40541;
    public static double shotSpeedIntercept = 675.67568;
    public static double shotSpeedMin       = 0.0;
    public static double shotSpeedMax       = 1500.0;

    // ── Internal State ────────────────────────────────────────────────────────
    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTimeNanos = System.nanoTime();
    private boolean isSpinning = false;

    public TurretShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftMotor  = hardwareMap.get(DcMotorEx.class, "turretShooterLeft");
        rightMotor = hardwareMap.get(DcMotorEx.class, "turretShooterRight");

        // Set zero power behavior to COAST for flywheels to prevent harsh motor braking
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // One motor reversed so both spin inward to propel the game piece
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastTimeNanos = System.nanoTime();
    }

    /**
     * Start spinning the shooter at a given target velocity (ticks/sec).
     */
    public void setTargetVelocity(double ticksPerSecond) {
        this.targetVelocity = Math.max(shotSpeedMin, Math.min(shotSpeedMax, ticksPerSecond));
        this.isSpinning = (targetVelocity > 0);
        if (!isSpinning) {
            integralSum = 0.0;
            lastError = 0.0;
        }
    }

    /**
     * Spin up for short-range shot (925 ticks/sec).
     */
    public void spinUpShort() {
        setTargetVelocity(outtakeVelocityShort);
    }

    /**
     * Spin up for long-range shot (1075 ticks/sec).
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
     * Stop the turret shooter motors immediately.
     */
    public void stop() {
        this.targetVelocity = 0.0;
        this.isSpinning = false;
        this.integralSum = 0.0;
        this.lastError = 0.0;
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public boolean isSpinning() {
        return isSpinning;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getAverageVelocity() {
        return currentVelocity;
    }

    @Override
    public void periodic() {
        long nowNanos = System.nanoTime();
        double dt = (nowNanos - lastTimeNanos) / 1e9;
        lastTimeNanos = nowNanos;

        if (dt <= 0 || dt > 0.2) dt = 0.02; // Guard against time jump

        // Read average actual motor velocity (ticks/second)
        double leftVel  = leftMotor.getVelocity();
        double rightVel = rightMotor.getVelocity();
        currentVelocity = (leftVel + rightVel) / 2.0;

        if (!isSpinning || targetVelocity <= 0) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            integralSum = 0.0;
            lastError = 0.0;
        } else {
            // PID error calculation
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
            double power = pidOutput + feedForward;
            power = Math.max(-1.0, Math.min(1.0, power));

            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }

        telemetry.addData("Shooter Target Vel (ticks/s)", targetVelocity);
        telemetry.addData("Shooter Actual Vel (ticks/s)", currentVelocity);
        telemetry.addData("Shooter Power", leftMotor.getPower());
    }
}
