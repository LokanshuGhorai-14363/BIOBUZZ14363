package Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.math.Pose;
import com.pedropathing.math.PoseFactory;

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
    private static final Pose RED_TARGET_POSE = PoseFactory.radians().of(120, 120, Math.toRadians(180));
    // Bottom Right (Blue Alliance)
    private static final Pose BLUE_TARGET_POSE = PoseFactory.radians().of(24, 24, 0);
    
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

    @Override
    public void execute() {
        // Path following is handled in DriveSubsystem.periodic() via follower.update()
        // If vision provides a valid corrected pose, we could inject it here to correct odometry mid-path.
        Pose correctedPose = visionSubsystem.getCorrectedBotPose();
        if (correctedPose != null) {
            driveSubsystem.setPose(correctedPose);
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
