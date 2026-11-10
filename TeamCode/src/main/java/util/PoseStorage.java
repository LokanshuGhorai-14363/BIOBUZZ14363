package util;

import com.pedropathing.math.Pose;

/**
 * PoseStorage — Static container for passing autonomous end pose to TeleOp.
 */
public class PoseStorage {
    /** Final robot pose saved by Autonomous routine for TeleOp handoff. */
    public static Pose currentPose = null;
}
