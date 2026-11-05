package opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import Subsystems.VisionSubsystem.Alliance;

/**
 * BlueVisionTeleOp — Single-Driver Competition TeleOp for Blue Alliance.
 *
 * <p>Extends {@link VisionTeleOp} with {@link Alliance#BLUE}. Automatically targets
 * the Blue Alliance HIVE Goals ((84, 60) Near / (84, 84) Far) with continuous
 * turret tracking and chassis auto-aim.</p>
 */
@TeleOp(name = "Blue Vision TeleOp", group = "Competition")
public class BlueVisionTeleOp extends VisionTeleOp {

    public BlueVisionTeleOp() {
        super(Alliance.BLUE);
    }
}
