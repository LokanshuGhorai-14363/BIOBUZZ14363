package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TurretSubsystem — Controls the spinning turret mechanism.
 *
 * Supports two modes:
 * 1. Closed-Loop CRServo + Analog Encoder (active when "turretEncoder" on Analog Port 1 is reading >0.05V).
 * 2. Standard Position Servo PWM mode (active when "turretServo" is set as a standard Servo on Servo Port 0).
 *
 * Range: 0° to 200° (100° is straight forward relative to the robot).
 */
public class TurretSubsystem extends SubsystemBase {

    private final Servo standardServo;
    private final CRServo crServo;
    private final AnalogInput turretEncoder;
    private final Telemetry telemetry;

    // Range & Calibration Constants
    public static final double MAX_PHYSICAL_DEGREES = 355.0;
    public static final double MIN_ANGLE_DEG = 0.0;
    public static final double MAX_ANGLE_DEG = 200.0;
    public static final double INITIAL_ANGLE_DEG = 100.0; // Straight forward relative to robot

    public static final double ENCODER_MAX_DEG = 360.0;
    public static final double MAX_VOLTAGE = 3.3;

    // Optional zero offset calibration (in degrees) if mechanical zero differs
    public static double ENCODER_ZERO_OFFSET_DEG = 0.0;

    // Set to true if motor power turns opposite to encoder angle increase
    public static boolean INVERT_SERVO = false;

    // PID gains for closed-loop turret feedback position control
    public static double kP = 0.025;
    public static double kI = 0.000;
    public static double kD = 0.001;
    private final PIDController pidController;

    private double currentTargetAngle = INITIAL_ANGLE_DEG;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.pidController = new PIDController(kP, kI, kD);

        Servo std = null;
        try {
            std = hardwareMap.get(Servo.class, "turretServo");
            if (std instanceof ServoImplEx) {
                ((ServoImplEx) std).setPwmRange(new PwmControl.PwmRange(500, 2500));
            }
        } catch (Exception ignored) {}

        CRServo cr = null;
        try {
            cr = hardwareMap.get(CRServo.class, "turretServo");
        } catch (Exception ignored) {}

        AnalogInput enc = null;
        try {
            enc = hardwareMap.get(AnalogInput.class, "turretEncoder");
        } catch (Exception ignored) {}

        this.standardServo = std;
        this.crServo = cr;
        this.turretEncoder = enc;

        // Initialize target angle to 100° (straight forward relative to robot)
        setAngle(INITIAL_ANGLE_DEG);
    }

    /**
     * Commands the turret to point at a specific target angle (clamped between 0° and 200°).
     *
     * @param degrees The target angle in degrees (100° = straight forward).
     */
    public void setAngle(double degrees) {
        currentTargetAngle = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, degrees));
    }

    /**
     * Reads the current measured absolute angle in degrees from the Axon MAX analog encoder wire.
     */
    public double getCurrentAngle() {
        if (turretEncoder != null) {
            double voltage = turretEncoder.getVoltage();
            if (voltage > 0.05) {
                double rawAngle = (voltage / MAX_VOLTAGE) * ENCODER_MAX_DEG;
                return (rawAngle + ENCODER_ZERO_OFFSET_DEG + 360.0) % 360.0;
            }
        }
        return currentTargetAngle;
    }

    @Override
    public void periodic() {
        double voltage = (turretEncoder != null) ? turretEncoder.getVoltage() : 0.0;

        // Use Closed-Loop PID only if turretEncoder is physically plugged in and reading > 0.05V
        if (turretEncoder != null && voltage > 0.05 && crServo != null) {
            double currentAngle = getCurrentAngle();
            double pidOutput = pidController.calculate(currentAngle, currentTargetAngle);

            if (INVERT_SERVO) pidOutput = -pidOutput;

            // Clamp motor power output to [-1.0, 1.0]
            double power = Math.max(-1.0, Math.min(1.0, pidOutput));

            // Deadband tolerance to eliminate motor jitter when close to target
            if (Math.abs(currentTargetAngle - currentAngle) < 0.5) {
                power = 0.0;
            }

            crServo.setPower(power);

            if (telemetry != null) {
                telemetry.addData("Turret Mode", "Closed-Loop CRServo + Analog Encoder");
                telemetry.addData("Encoder Voltage (V)", "%.2f", voltage);
                telemetry.addData("Turret Measured Angle (deg)", "%.1f", currentAngle);
                telemetry.addData("Turret Target Angle (deg)", "%.1f", currentTargetAngle);
                telemetry.addData("Turret Servo Power", "%.2f", power);
            }
        } else if (standardServo != null) {
            // Standard Position Servo Mode (0.0 to 1.0 position)
            double servoPos = currentTargetAngle / MAX_PHYSICAL_DEGREES;
            if (INVERT_SERVO) {
                servoPos = 1.0 - servoPos;
            }
            standardServo.setPosition(Math.max(0.0, Math.min(1.0, servoPos)));

            if (telemetry != null) {
                telemetry.addData("Turret Mode", "Standard Position Servo PWM");
                telemetry.addData("Turret Target Angle (deg)", "%.1f", currentTargetAngle);
                telemetry.addData("Turret Servo Position", "%.3f", servoPos);
                if (turretEncoder == null) {
                    telemetry.addData(">> NOTICE", "Encoder 'turretEncoder' not found -> Using Standard Positional Servo");
                } else {
                    telemetry.addData(">> NOTICE", String.format("Encoder Voltage = %.2fV -> Using Standard Positional Servo", voltage));
                }
            }
        } else {
            if (telemetry != null) {
                telemetry.addData(">> TURRET ERROR", "'turretServo' not found in Driver Station config!");
            }
        }
    }
}
