package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * FlowerSubsystem — Controls the flower servo ("flowerServo").
 *
 * Rest/Stowed state (POS_0_DEG = 0.55): Raised UP.
 * Deployed state (POS_90_DEG = 0.00): Lowered DOWN (~10° below horizontal to reach flower).
 */
public class FlowerSubsystem extends SubsystemBase {

    // Servo positions & direction (0.55 = UP/Raised rest state, 0.00 = DOWN/Lowered state)
    public static Servo.Direction SERVO_DIRECTION = Servo.Direction.FORWARD;
    public static double POS_0_DEG  = 0.55; // UP / Raised position
    public static double POS_90_DEG = 0.00; // DOWN / Lowered position (~10° below horizontal)

    private final Servo flowerServo;
    private final Telemetry telemetry;

    private boolean is90Deg = false;

    public FlowerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "flowerServo");
    }

    public FlowerSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        this.telemetry = telemetry;
        this.flowerServo = hardwareMap.get(Servo.class, servoName);

        if (flowerServo != null) {
            if (flowerServo instanceof ServoImplEx) {
                ((ServoImplEx) flowerServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
            }
            flowerServo.setDirection(SERVO_DIRECTION);
            flowerServo.setPosition(POS_0_DEG);
        }
    }

    /**
     * Toggles position between Raised UP and Lowered DOWN.
     */
    public void toggle() {
        if (is90Deg) {
            setAngle0();
        } else {
            setAngle90();
        }
    }

    /**
     * Lower flower servo DOWN to intake flower (0.00).
     */
    public void setAngle90() {
        is90Deg = true;
        if (flowerServo != null) {
            flowerServo.setPosition(Math.max(0.0, Math.min(1.0, POS_90_DEG)));
        }
    }

    /**
     * Raise flower servo UP to rest position (0.55).
     */
    public void setAngle0() {
        is90Deg = false;
        if (flowerServo != null) {
            flowerServo.setPosition(Math.max(0.0, Math.min(1.0, POS_0_DEG)));
        }
    }

    public boolean is90Deg() {
        return is90Deg;
    }

    public double getPosition() {
        return (flowerServo != null) ? flowerServo.getPosition() : 0.0;
    }

    @Override
    public void periodic() {
        if (telemetry != null && flowerServo != null) {
            telemetry.addData("Flower Servo Found", true);
            telemetry.addData("Flower Servo State", is90Deg ? "DOWN (Deployed)" : "UP (Stowed)");
            telemetry.addData("Flower Servo Position", String.format("%.2f", flowerServo.getPosition()));
        }
    }
}
