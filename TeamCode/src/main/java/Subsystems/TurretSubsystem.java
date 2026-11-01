package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TurretSubsystem — Closed-loop turret positioning using an Axon MAX Mark 2
 * analog absolute encoder on Analog Port 1 and CRServo on Servo Port 0.
 *
 * Range: 0° to 200° (100° is straight forward relative to the robot).
 */
public class TurretSubsystem extends SubsystemBase {

    private final CRServo turretServo;
    private final AnalogInput turretEncoder;
    private final Telemetry telemetry;

    // Range & Calibration Constants
    public static final double MIN_ANGLE_DEG = 0.0;
    public static final double MAX_ANGLE_DEG = 200.0;
    public static final double INITIAL_ANGLE_DEG = 100.0; // Straight forward relative to robot

    public static final double ENCODER_MAX_DEG = 360.0;
    public static final double MAX_VOLTAGE = 3.3;

    // Optional zero offset calibration (in degrees) if mechanical zero differs
    public static double ENCODER_ZERO_OFFSET_DEG = 0.0;

    // PID gains for closed-loop turret feedback position control
    public static double kP = 0.025;
    public static double kI = 0.000;
    public static double kD = 0.001;
    private final PIDController pidController;

    private double currentTargetAngle = INITIAL_ANGLE_DEG;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.pidController = new PIDController(kP, kI, kD);

        this.turretServo = hardwareMap.get(CRServo.class, "turretServo");
        this.turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

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
            double rawAngle = (voltage / MAX_VOLTAGE) * ENCODER_MAX_DEG;
            return (rawAngle + ENCODER_ZERO_OFFSET_DEG + 360.0) % 360.0;
        }
        return currentTargetAngle;
    }

    @Override
    public void periodic() {
        double currentAngle = getCurrentAngle();
        double pidOutput = pidController.calculate(currentAngle, currentTargetAngle);

        // Clamp motor power output to [-1.0, 1.0]
        double power = Math.max(-1.0, Math.min(1.0, pidOutput));

        // Deadband tolerance to eliminate motor jitter when close to target
        if (Math.abs(currentTargetAngle - currentAngle) < 0.5) {
            power = 0.0;
        }

        turretServo.setPower(power);

        telemetry.addData("Turret Mode", "Closed-Loop CRServo + Analog Encoder");
        telemetry.addData("Turret Measured Angle (deg)", currentAngle);
        telemetry.addData("Turret Target Angle (deg)", currentTargetAngle);
        telemetry.addData("Turret Servo Power", power);
    }
}
