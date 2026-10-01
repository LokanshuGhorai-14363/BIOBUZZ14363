package opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import Subsystems.VisionSubsystem.Alliance;

/**
 * RedVisionTeleOp — TeleOp configuration for Red Alliance.
 */
@TeleOp(name = "Red Vision TeleOp", group = "Competition")
public class RedVisionTeleOp extends VisionTeleOp {

    public RedVisionTeleOp() {
        super(Alliance.RED);
    }
}
