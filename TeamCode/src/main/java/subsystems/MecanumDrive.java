package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;

import utility.RobotHardware;

/**
 * Four-wheel mecanum drive train. Pedro 3 owns the drive motors; this subsystem only hands
 * the requested powers to the Follower, which applies them on its next update(). Never set
 * motor power directly while a Follower exists (rule 16).
 *
 * Powers are robot-centric and use Pedro's sign convention: +forward, +strafe to the LEFT,
 * +turn counter-clockwise. Callers are responsible for any field-centric rotation.
 */
public class MecanumDrive extends SubsystemBase {
    private final Follower follower;

    public MecanumDrive() {
        this.follower = RobotHardware.getInstance().follower;
    }

    public void drive(double forward, double strafe, double turn) {
        follower.manual(forward, strafe, turn);
    }

    public void stop() {
        drive(0, 0, 0);
    }
}
