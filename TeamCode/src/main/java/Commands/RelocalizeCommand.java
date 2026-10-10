package Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.math.Pose;

import Subsystems.DriveSubsystem;
import Subsystems.VisionSubsystem;
import Subsystems.VisionSubsystem.Alliance;

/**
 * RelocalizeCommand — Autonomously drives to a designated corner based on the alliance.
 * Interrupts TeleOp driving when executed.
 */
public class RelocalizeCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    
    // Top Left (Red Alliance)
    private static final Pose RED_TARGET_POSE = new Pose(120, 120, Math.toRadians(180));
    // Bottom Right (Blue Alliance)
    private static final Pose BLUE_TARGET_POSE = new Pose(24, 24, 0);
    
    private final Alliance currentAlliance;

    public RelocalizeCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Alliance alliance) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.currentAlliance = alliance;
        
        // Requires drive subsystem to interrupt joystick driving
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        Pose target = (currentAlliance == Alliance.RED) ? RED_TARGET_POSE : BLUE_TARGET_POSE;
        driveSubsystem.followPathTo(target);
    }

    private long lastRelocalizeTime = 0;

    @Override
    public void execute() {
        // Limit relocalization to 10Hz to prevent excessive odometry jitter and reduce loop times
        if (System.currentTimeMillis() - lastRelocalizeTime > 100) {
            Pose correctedPose = visionSubsystem.getCorrectedBotPose();
            if (correctedPose != null) {
                driveSubsystem.setPose(correctedPose);
                lastRelocalizeTime = System.currentTimeMillis();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.isPathComplete();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.resumeTeleOp();
    }
}
