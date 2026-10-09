package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * IntakeSubsystem — controls both the intake motor and transfer motor.
 *
 * <h3>Hardware</h3>
 * <ul>
 *   <li>{@code intakeMotor} — goBILDA Yellow Jacket 1100 RPM intake motor.</li>
 *   <li>{@code transferMotor} — single transfer motor driving game elements
 *       from the intake directly to the scoring/shooting area.</li>
 * </ul>
 *
 * Positive power = intake/transfer IN  (collect cargo & transfer to shooter)
 * Negative power = intake/transfer OUT (reverse / eject cargo)
 */
public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private final DcMotorEx transferMotor;
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

        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
     * Spin both intake and transfer motors outward (reverse) to eject cargo.
     */
    public void spinOut() {
        setPower(-MAX_OUT_POWER, -MAX_OUT_POWER);
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

    // ───────────────────────────── Accessors ───────────────────────────────────

    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }

    public DcMotorEx getTransferMotor() {
        return transferMotor;
    }

    // ───────────────────────────── Lifecycle ───────────────────────────────────

    @Override
    public void periodic() {
        telemetry.addData("Intake Power", intakeMotor.getPower());
        telemetry.addData("Transfer Power", transferMotor.getPower());
    }
}
