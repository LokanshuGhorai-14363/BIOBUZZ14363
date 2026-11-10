package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * FlowerSubsystem — Controls the flower servo ("flowerServo").
 *
 * Toggles position between 0° (0.0) and 90° (0.5).
 */
public class FlowerSubsystem extends SubsystemBase {

    // Servo positions (normalized 0.0 to 1.0; 0.5 = 90° for standard 180° servo)
    public static double POS_0_DEG  = 0.0;
    public static double POS_90_DEG = 0.5;

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
            flowerServo.setPosition(POS_0_DEG);
        }
    }

    /**
     * Toggles position between 0° and 90°.
     */
    public void toggle() {
        if (is90Deg) {
            setAngle0();
        } else {
            setAngle90();
        }
    }

    /**
     * Sets position to 90 degrees (Counter-Clockwise).
     */
    public void setAngle90() {
        is90Deg = true;
        if (flowerServo != null) {
            flowerServo.setPosition(POS_90_DEG);
        }
    }

    /**
     * Sets position to 0 degrees (Clockwise).
     */
    public void setAngle0() {
        is90Deg = false;
        if (flowerServo != null) {
            flowerServo.setPosition(POS_0_DEG);
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
            telemetry.addData("Flower Servo Angle", is90Deg ? "90° (CCW)" : "0° (CW)");
            telemetry.addData("Flower Servo Position", String.format("%.2f", flowerServo.getPosition()));
        }
    }
}
