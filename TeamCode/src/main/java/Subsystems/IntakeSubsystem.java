package Subsystems;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * IntakeSubsystem — controls both the intake motor and transfer motor with automated color sensing.
 *
 * <h3>Hardware</h3>
 * <ul>
 *   <li>{@code intakeMotor} — goBILDA Yellow Jacket 1100 RPM intake motor ("intakeMotor").</li>
 *   <li>{@code transferMotor} — single transfer motor driving game elements ("transferMotor").</li>
 *   <li>{@code colorSensors} — 4 NormalizedColorSensor instances placed sequentially along transfer ("colorSensor1".."colorSensor4").</li>
 * </ul>
 *
 * <h3>Target Game Elements</h3>
 * <ul>
 *   <li>Pollen (Yellow): #FFC82E</li>
 *   <li>Nectar Red: #ED1C24</li>
 *   <li>Nectar Blue: #0066B3</li>
 * </ul>
 */
public class IntakeSubsystem extends SubsystemBase {

    public enum BallColor {
        NONE,
        YELLOW,
        RED,
        BLUE
    }

    private final DcMotorEx intakeMotor;
    private final DcMotorEx transferMotor;
    private final NormalizedColorSensor[] colorSensors = new NormalizedColorSensor[4];
    private final Telemetry telemetry;

    /** Maximum intake collection power (0–1). */
    public static final double MAX_IN_POWER     = 1.0;
    /** Maximum reverse/eject power (0–1). */
    public static final double MAX_OUT_POWER    = 0.8;
    /** Full reverse power for auto-ejection. */
    public static final double FULL_EJECT_POWER = 1.0;

    // ── Color Sensor Thresholds ──────────────────────────────────────────────
    public static float COLOR_SENSOR_GAIN         = 3.0f;
    public static float MIN_ALPHA_THRESHOLD       = 0.015f;
    public static float MIN_TOTAL_LIGHT_THRESHOLD = 0.03f;
    public static float MIN_SATURATION            = 0.25f;

    /** Illegal ball color that triggers auto-eject sequence. */
    public static BallColor ILLEGAL_COLOR         = BallColor.BLUE;

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     */
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (int i = 0; i < 4; i++) {
            String sensorName = "colorSensor" + (i + 1);
            try {
                colorSensors[i] = hardwareMap.get(NormalizedColorSensor.class, sensorName);
                if (colorSensors[i] != null) {
                    colorSensors[i].setGain(COLOR_SENSOR_GAIN);
                }
            } catch (Exception e) {
                if (telemetry != null) {
                    telemetry.addData("Warning", "Sensor missing: " + sensorName);
                }
            }
        }
    }

    // ───────────────────────────── Actions ─────────────────────────────────────

    /**
     * Spin both intake and transfer motors inward to collect cargo and feed the shooter.
     *
     * @param analogPower value from 0.0–1.0 (e.g. trigger axis)
     */
    public void spinIn(double analogPower) {
        double power = Math.abs(analogPower) * MAX_IN_POWER;
        setPower(power, power);
    }

    /**
     * Spin both intake and transfer motors outward (reverse) at standard eject power.
     */
    public void spinOut() {
        setPower(-MAX_OUT_POWER, -MAX_OUT_POWER);
    }

    /**
     * Spin both intake and transfer motors in reverse at FULL power for automated purge.
     */
    public void spinOutFull() {
        setPower(-FULL_EJECT_POWER, -FULL_EJECT_POWER);
    }

    /**
     * Set power for both intake and transfer motors together.
     *
     * @param intakePower   power level for the intake motor (-1.0 to 1.0)
     * @param transferPower power level for the transfer motor (-1.0 to 1.0)
     */
    public void setPower(double intakePower, double transferPower) {
        intakeMotor.setPower(intakePower);
        transferMotor.setPower(transferPower);
    }

    /**
     * Stop both intake and transfer motors.
     */
    public void stop() {
        setPower(0.0, 0.0);
    }

    // ───────────────────────────── Color Sensing & Logic ───────────────────────

    /**
     * Classifies a color sensor's reading into a {@link BallColor}.
     *
     * @param sensor the color sensor to evaluate
     * @return BallColor enum (NONE, YELLOW, RED, or BLUE)
     */
    public BallColor classifyColor(NormalizedColorSensor sensor) {
        if (sensor == null) {
            return BallColor.NONE;
        }

        NormalizedRGBA colors = sensor.getNormalizedColors();
        if (colors == null) {
            return BallColor.NONE;
        }

        float totalLight = colors.red + colors.green + colors.blue;
        if (colors.alpha < MIN_ALPHA_THRESHOLD && totalLight < MIN_TOTAL_LIGHT_THRESHOLD) {
            return BallColor.NONE;
        }

        float[] hsv = new float[3];
        Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );

        float hue        = hsv[0]; // 0..360
        float saturation = hsv[1]; // 0..1

        if (saturation < MIN_SATURATION) {
            return BallColor.NONE;
        }

        // Pollen (Yellow): #FFC82E -> Hue ~ 30° - 70°
        if (hue >= 30.0f && hue <= 70.0f) {
            return BallColor.YELLOW;
        }

        // Nectar Red: #ED1C24 -> Hue ~ 340° - 360° or 0° - 25°
        if ((hue >= 340.0f && hue <= 360.0f) || (hue >= 0.0f && hue <= 25.0f)) {
            return BallColor.RED;
        }

        // Nectar Blue: #0066B3 -> Hue ~ 180° - 240°
        if (hue >= 180.0f && hue <= 240.0f) {
            return BallColor.BLUE;
        }

        return BallColor.NONE;
    }

    /**
     * Reads the classified color from a sensor by index (0 to 3, or 1 to 4).
     *
     * @param index sensor index (0..3 or 1..4)
     * @return BallColor detected at the given sensor
     */
    public BallColor getSensorColor(int index) {
        int idx = (index >= 1 && index <= 4) ? index - 1 : index;
        if (idx < 0 || idx >= colorSensors.length) {
            return BallColor.NONE;
        }
        return classifyColor(colorSensors[idx]);
    }

    /**
     * @return {@code true} if ALL 4 sensors detect a ball (capacity limit reached).
     */
    public boolean isFull() {
        for (int i = 0; i < 4; i++) {
            if (getSensorColor(i) == BallColor.NONE) {
                return false;
            }
        }
        return true;
    }

    /**
     * @return {@code true} if ALL 4 sensors report {@link BallColor#NONE} (completely empty).
     */
    public boolean areAllSensorsEmpty() {
        for (int i = 0; i < 4; i++) {
            if (getSensorColor(i) != BallColor.NONE) {
                return false;
            }
        }
        return true;
    }

    /**
     * Checks if any sensor detects the illegal ball color (default: BLUE).
     *
     * @return {@code true} if an ejection/purge sequence should be triggered.
     */
    public boolean shouldEject() {
        for (int i = 0; i < 4; i++) {
            if (getSensorColor(i) == ILLEGAL_COLOR) {
                return true;
            }
        }
        return false;
    }

    // ───────────────────────────── Accessors ───────────────────────────────────

    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public DcMotorEx getTransferMotor() {
        return transferMotor;
    }

    public NormalizedColorSensor getSensor(int index) {
        int idx = (index >= 1 && index <= 4) ? index - 1 : index;
        if (idx < 0 || idx >= colorSensors.length) {
            return null;
        }
        return colorSensors[idx];
    }

    // ───────────────────────────── Lifecycle ───────────────────────────────────

    @Override
    public void periodic() {
        if (telemetry != null) {
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addData("Transfer Power", transferMotor.getPower());
            for (int i = 0; i < 4; i++) {
                telemetry.addData("Sensor " + (i + 1), getSensorColor(i));
            }
            telemetry.addData("Transfer Full", isFull());
            telemetry.addData("Eject Active", shouldEject());
        }
    }
}
