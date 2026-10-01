package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TurretSubsystem — Controls the spinning turret mechanism.
 *
 * Hardware: 1 Axon MAX Mark 2 servo ("turretServo").
 * The servo is clamped to a physical rotation range of 0 to 220 degrees.
 */
public class TurretSubsystem extends SubsystemBase {

    private final Servo turretServo;
    private final Telemetry telemetry;

    // Constants for scaling
    // Assuming the Axon MAX servo's 0.0 to 1.0 PWM range maps to 0 to 355 degrees (standard Axon continuous).
    // Adjust MAX_PHYSICAL_DEGREES if the programmer set it differently.
    public static final double MAX_PHYSICAL_DEGREES = 355.0; 
    public static final double MIN_ANGLE_DEG = 0.0;
    public static final double MAX_ANGLE_DEG = 220.0;
    
    private double currentTargetAngle = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.turretServo = hardwareMap.get(Servo.class, "turretServo");
        
        // Initialize to 0 degrees
        setAngle(0.0);
    }

    /**
     * Commands the turret to point at a specific target angle.
     * The angle is clamped strictly between 0 and 220 degrees.
     *
     * @param degrees The target angle in degrees.
     */
    public void setAngle(double degrees) {
        // Clamp the angle
        currentTargetAngle = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, degrees));
        
        // Scale to 0.0 - 1.0 range for the servo
        double servoPosition = currentTargetAngle / MAX_PHYSICAL_DEGREES;
        turretServo.setPosition(servoPosition);
    }

    /**
     * @return The current target angle in degrees.
     */
    public double getCurrentAngle() {
        return currentTargetAngle;
    }

    @Override
    public void periodic() {
        telemetry.addData("Turret Angle (deg)", currentTargetAngle);
        telemetry.addData("Turret Servo Pos", turretServo.getPosition());
    }
}
