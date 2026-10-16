package Constants;

import com.pedropathing.math.Pose;

/**
 * Field poses shared across autos and TeleOp. Pedro coordinates are inches on the
 * 144x144 field, headings in radians.
 */
public class OdometryConstants {
    // TODO (needs robot test): placeholder poses. Pose.zero() is the field corner, which
    // is not a legal start position - re-measure both against the real field before any
    // match. TeleOpTemplate falls back to these when endingAutonPose is null.
    public static Pose redStartPoint = Pose.zero();
    public static Pose blueStartPoint = Pose.zero();

    /**
     * Pose carried from the end of auto into TeleOp, read by TeleOpTemplate.
     *
     * TODO: nothing writes this yet - the auto side (AutonTemplate.stop() assigning
     * follower.pose()) has not been written, so it is always null and TeleOp always uses
     * the fallback start pose. Note for when that lands: statics live as long as the
     * Robot Controller app, and nothing clears this, so every TeleOp run after the first
     * auto of the session restores the auto's ending pose - including practice runs
     * started from the wall. Decide then whether TeleOp should consume it (read, then
     * null it) or whether auto init should clear it.
     */
    public static Pose endingAutonPose = null;
}
