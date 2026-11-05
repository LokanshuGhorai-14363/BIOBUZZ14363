package opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import Subsystems.VisionSubsystem.Alliance;

/**
 * MainTeleOp — Primary Single-Driver TeleOp for Red Alliance.
 *
 * <p>Inherits from {@link VisionTeleOp} with {@link Alliance#RED}. Features
 * field-centric driving, auto-aim to HIVE goals, automated Pinpoint odometry
 * turret tracking, distance-scaled shooter flywheel control, and one-touch
 * rapid scoring macros on Gamepad 1.</p>
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends VisionTeleOp {

    public MainTeleOp() {
        super(Alliance.RED);
    }
}
