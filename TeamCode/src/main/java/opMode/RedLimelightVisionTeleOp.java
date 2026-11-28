package opMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import Subsystems.VisionSubsystem.Alliance;

/**
 * RedLimelightVisionTeleOp — Competition TeleOp for Red Alliance using Limelight 3A vision aiming.
 *
 * <p>Uses Limelight 3A (mounted at X=+4.913", Y=-6.0965", Z=+6.500") for active target tracking
 * with seamless fallback to Pinpoint Odometry.</p>
 */
@TeleOp(name = "Red Limelight Vision TeleOp", group = "Competition")
public class RedLimelightVisionTeleOp extends LimelightVisionTeleOp {

    public RedLimelightVisionTeleOp() {
        super(Alliance.RED);
    }
}
