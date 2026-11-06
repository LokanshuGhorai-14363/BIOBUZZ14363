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
 * Hardware: 1 Axon MAX Mark 2 Servo ("turretServo") on Servo Port 0.
 * PWM Range: 500 µs to 2500 µs.
 * Center position: 0.50 (1500 µs) = 100° (straight forward).
 */
public class TurretSubsystem extends SubsystemBase {

    private final Servo turretServo;
    private final Telemetry telemetry;

    // Constants for physical turret sweep limits
    public static final double MIN_ANGLE_DEG = 0.0;     // 100° Left of center
    public static final double MAX_ANGLE_DEG = 200.0;   // 100° Right of center
    public static final double INITIAL_ANGLE_DEG = 100.0; // Straight forward center

    // Axon MAX Mark 2 Servo physical rotation range across 500µs - 2500µs
    public static double SERVO_PHYSICAL_RANGE_DEG = 355.0;

    /**
     * TURRET GEAR RATIO = (Turret Gear Teeth / Servo Gear Teeth)
     * Example: 60-tooth turret ring gear driven by a 20-tooth servo gear -> Ratio = 3.0.
     * If direct drive (1:1), set to 1.0.
     */
    public static double TURRET_GEAR_RATIO = 1.0;

    /**
     * SERVO CENTER POSITION (0.0 to 1.0)
     * The PWM position corresponding to 100° (straight forward center).
     * Standard PWM center is 0.50 (1500 µs).
     */
    public static double SERVO_CENTER_POSITION = 0.50;

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
     * Maps 100° (center) to SERVO_CENTER_POSITION.
     * Accounts for turret gear ratio reduction.
     */
    private void updateServoPosition() {
        // Degrees relative to center (100°)
        double deltaTurretDeg = currentTargetAngle - INITIAL_ANGLE_DEG;

        // Account for gear ratio (e.g., if turret gear is 3x larger than servo gear, servo must turn 3x more)
        double deltaServoDeg = deltaTurretDeg * TURRET_GEAR_RATIO;

        if (INVERT_SERVO) {
            deltaServoDeg = -deltaServoDeg;
        }

        // Add offset relative to SERVO_CENTER_POSITION
        double servoPos = SERVO_CENTER_POSITION + (deltaServoDeg / SERVO_PHYSICAL_RANGE_DEG);

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
