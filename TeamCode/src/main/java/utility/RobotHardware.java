package utility;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Singleton hardware holder. Pedro 3 owns the Pinpoint and drive motors through the
 * Follower it builds; this class only creates the Follower and caches its pose/velocity
 * once per loop. Never write power to the drive motors or reconfigure the Pinpoint outside
 * of Pedro.
 */
public class RobotHardware {
    private static volatile RobotHardware instance;

    public Follower follower;
    private List<LynxModule> allHubs;

    public double cachedPoseX;
    public double cachedPoseY;
    public double cachedPoseHeading;
    public double cachedVelX;
    public double cachedVelY;
    public double cachedVelHeading;

    private RobotHardware() {
    }

    public static RobotHardware getInstance() {
        if (instance == null) {
            synchronized (RobotHardware.class) {
                if (instance == null) {
                    instance = new RobotHardware();
                }
            }
        }
        return instance;
    }

    public void init(HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = pedroPathing.Constants.createFollower(hardwareMap);
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void updateCachedPose() {
        Pose pose = follower.pose();
        cachedPoseX = pose.x();
        cachedPoseY = pose.y();
        cachedPoseHeading = pose.heading();

        Velocity velocity = follower.velocity();
        cachedVelX = velocity.vx;
        cachedVelY = velocity.vy;
        cachedVelHeading = velocity.omega;
    }
}
