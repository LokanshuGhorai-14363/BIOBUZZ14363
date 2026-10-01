package opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import Subsystems.VisionSubsystem.Alliance;

/**
 * BlueVisionTeleOp — TeleOp configuration for Blue Alliance.
 */
@TeleOp(name = "Blue Vision TeleOp", group = "Competition")
public class BlueVisionTeleOp extends VisionTeleOp {

    public BlueVisionTeleOp() {
        super(Alliance.BLUE);
    }
}
