package opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import Subsystems.VisionSubsystem.Alliance;

/**
 * BlueLimelightVisionTeleOp — Competition TeleOp for Blue Alliance using Limelight 3A vision aiming.
 *
 * <p>Uses Limelight 3A (mounted at X=+4.913", Y=-6.0965", Z=+6.500") for active target tracking
 * with seamless fallback to Pinpoint Odometry.</p>
 */
@TeleOp(name = "Blue Limelight Vision TeleOp", group = "Competition")
public class BlueLimelightVisionTeleOp extends LimelightVisionTeleOp {

    public BlueLimelightVisionTeleOp() {
        super(Alliance.BLUE);
    }
}
