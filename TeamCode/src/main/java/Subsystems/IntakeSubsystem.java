package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * IntakeSubsystem — single goBILDA Yellow Jacket 1100 RPM motor.
 *
 * Positive power  = intake IN  (collect cargo)
 * Negative power  = intake OUT (reverse / eject)
 */
public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private final Telemetry telemetry;

    /** Maximum intake collection power (0–1). */
    public static final double MAX_IN_POWER  = 1.0;
    /** Maximum reverse/eject power (0–1). */
    public static final double MAX_OUT_POWER = 0.8;

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     */
    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ───────────────────────────── Actions ─────────────────────────────────────

    /**
     * Spin the intake inward to collect cargo.
     *
     * @param analogPower value from 0.0–1.0 (e.g. trigger axis)
     */
    public void spinIn(double analogPower) {
        intakeMotor.setPower(Math.abs(analogPower) * MAX_IN_POWER);
    }

    /**
     * Spin the intake outward (reverse) to eject cargo.
     */
    public void spinOut() {
        intakeMotor.setPower(-MAX_OUT_POWER);
    }

    /**
     * Stop the intake motor.
     */
    public void stop() {
        intakeMotor.setPower(0);
    }

    // ───────────────────────────── Lifecycle ───────────────────────────────────

    @Override
    public void periodic() {
        telemetry.addData("Intake Power", intakeMotor.getPower());
    }
}
