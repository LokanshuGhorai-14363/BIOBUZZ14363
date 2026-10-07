package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;

import utility.RobotHardware;

/**
 * Four-wheel mecanum drive train. Pedro 3 owns the drive motors; this subsystem only
 * caches the latest requested drive power and forwards it to the Follower each loop via
 * follower.manual(...). Never set motor power directly while a Follower exists (rule 16).
 */
public class MecanumDrive extends SubsystemBase {
    private final Follower follower;

    private double lastForward = 0;
    private double lastStrafe = 0;
    private double lastTurn = 0;

    public MecanumDrive() {
        this.follower = RobotHardware.getInstance().follower;
    }

    public void drive(double forward, double strafe, double turn) {
        lastForward = forward;
        lastStrafe = strafe;
        lastTurn = turn;
    }

    public void stop() {
        drive(0, 0, 0);
    }

    @Override
    public void periodic() {
        follower.manual(lastForward, lastStrafe, lastTurn);
    }
}
