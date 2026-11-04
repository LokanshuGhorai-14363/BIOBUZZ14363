package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * TurretSubsystem — Standard Positional Servo Control for Turret Mechanism.
 *
 * Hardware: 1 Standard Servo ("turretServo") on Servo Port 0.
 * PWM Range: 500 µs to 2500 µs (maps to 0° - 355° physical range).
 * Sweep Limits: 0° to 200° (100° is straight forward center, allowing 100° Left and 100° Right).
 */
public class TurretSubsystem extends SubsystemBase {

    private final Servo turretServo;
    private final Telemetry telemetry;

    // Constants for physical servo limits and scaling
    public static final double MAX_PHYSICAL_DEGREES = 355.0;
    public static final double MIN_ANGLE_DEG = 0.0;     // 100° Left of center
    public static final double MAX_ANGLE_DEG = 200.0;   // 100° Right of center
    public static final double INITIAL_ANGLE_DEG = 100.0; // Straight forward center

    // Standard Axon MAX Mark 2 Servo PWM limits (in microseconds)
    public static final int PWM_MIN_US = 500;
    public static final int PWM_MAX_US = 2500;

    // Invert servo movement direction if servo turns CW instead of CCW
    public static boolean INVERT_SERVO = false;

    private double currentTargetAngle = INITIAL_ANGLE_DEG;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        this.turretServo = hardwareMap.get(Servo.class, "turretServo");

        // Expand PWM range to 500µs - 2500µs for full Axon MAX 355° range
        if (turretServo instanceof ServoImplEx) {
            ((ServoImplEx) turretServo).setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));
        }

        // On initialization, command servo to 100° (straight forward center)
        setAngle(INITIAL_ANGLE_DEG);
    }

    /**
     * Commands the turret to point at a specific target angle.
     * Clamped strictly between 0° (100° Left) and 200° (100° Right).
     *
     * @param degrees The target angle in degrees (100° = straight forward).
     */
    public void setAngle(double degrees) {
        currentTargetAngle = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, degrees));
        updateServoPosition();
    }

    /**
     * Internal helper to calculate and output the PWM position to the servo.
     */
    private void updateServoPosition() {
        double servoPos = currentTargetAngle / MAX_PHYSICAL_DEGREES;
        if (INVERT_SERVO) {
            servoPos = 1.0 - servoPos;
        }
        turretServo.setPosition(Math.max(0.0, Math.min(1.0, servoPos)));
    }

    /**
     * @return The current target angle in degrees.
     */
    public double getCurrentAngle() {
        return currentTargetAngle;
    }

    @Override
    public void periodic() {
        updateServoPosition();

        if (telemetry != null) {
            telemetry.addData("Turret Mode", "Standard Position Servo PWM");
            telemetry.addData("Turret Target Angle (deg)", "%.1f", currentTargetAngle);
            telemetry.addData("Turret Servo Position", "%.3f", turretServo.getPosition());
        }
    }
}
