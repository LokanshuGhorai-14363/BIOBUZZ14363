package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * IntakeSubsystem — high-performance proportional control of the intake motor.
 *
 * <h3>Hardware Ownership</h3>
 * Exclusive owner of {@code intakeMotor} ("intakeMotor"). Eliminates bus contention
 * with {@link TransferSubsystem}.
 */
public class IntakeSubsystem extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private final Telemetry telemetry;

    public static final double MAX_IN_POWER  = 1.0;
    public static final double MAX_OUT_POWER = 1.0;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0.0);
    }

    public void spinIn(double analogPower) {
        double power = Math.max(0.0, Math.min(1.0, Math.abs(analogPower))) * MAX_IN_POWER;
        setPower(power);
    }

    public void spinOut(double analogPower) {
        double power = Math.max(0.0, Math.min(1.0, Math.abs(analogPower))) * MAX_OUT_POWER;
        setPower(-power);
    }

    public void spinOutFull() {
        spinOut(1.0);
    }

    public void setPower(double intakePower) {
        intakeMotor.setPower(intakePower);
    }

    public void stop() {
        setPower(0.0);
    }

    public DcMotorEx getIntakeMotor() {
        return intakeMotor;
    }

    @Override
    public void periodic() {
        if (telemetry != null && intakeMotor != null) {
            telemetry.addData("Intake Power", intakeMotor.getPower());
        }
    }
}
