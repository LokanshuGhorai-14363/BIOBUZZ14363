package opMode;

import Commands.LimelightTurretTrackCommand;
import Subsystems.LimelightSubsystem;
import Subsystems.VisionSubsystem.Alliance;

/**
 * LimelightVisionTeleOp — Base TeleOp class utilizing Limelight 3A for vision aiming.
 */
public abstract class LimelightVisionTeleOp extends VisionTeleOp {

    protected LimelightSubsystem limelightSubsystem;

    public LimelightVisionTeleOp(Alliance alliance) {
        super(alliance);
    }

    @Override
    public void initialize() {
        limelightSubsystem = new LimelightSubsystem(hardwareMap, telemetry);
        super.initialize();
        register(limelightSubsystem);
    }

    @Override
    protected void configureDefaultCommands() {
        super.configureDefaultCommands();
        if (limelightSubsystem != null && turretSubsystem != null) {
            turretSubsystem.setDefaultCommand(new LimelightTurretTrackCommand(
                    turretSubsystem,
                    limelightSubsystem,
                    visionSubsystem,
                    () -> manualTurretMode,
                    () -> coDriverGamepad.getLeftX()
            ));
        }
    }
}
