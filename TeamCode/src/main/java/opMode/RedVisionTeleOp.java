package opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import Subsystems.VisionSubsystem.Alliance;

/**
 * RedVisionTeleOp — Single-Driver Competition TeleOp for Red Alliance.
 *
 * <p>Extends {@link VisionTeleOp} with {@link Alliance#RED}. Automatically targets
 * the Red Alliance HIVE Goals ((60, 60) Near / (60, 84) Far) with continuous
 * turret tracking and chassis auto-aim.</p>
 */
@TeleOp(name = "Red Vision TeleOp", group = "Competition")
public class RedVisionTeleOp extends VisionTeleOp {

    public RedVisionTeleOp() {
        super(Alliance.RED);
    }
}
