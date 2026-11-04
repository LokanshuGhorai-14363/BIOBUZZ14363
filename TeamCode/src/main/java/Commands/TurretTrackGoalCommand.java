package Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import Subsystems.TurretSubsystem;
import Subsystems.VisionSubsystem;

/**
 * TurretTrackGoalCommand — Continuous default command for TurretSubsystem.
 *
 * Dynamically calculates the required turret angle every loop iteration
 * using Pinpoint odometry (via VisionSubsystem) to point the turret continuously
 * at the active alliance goal while the robot is stationary or driving.
 */
public class TurretTrackGoalCommand extends CommandBase {

    private final TurretSubsystem turretSubsystem;
    private final VisionSubsystem visionSubsystem;

    /**
     * @param turretSubsystem the turret subsystem to command
     * @param visionSubsystem the vision subsystem for odometry goal angle calculation
     */
    public TurretTrackGoalCommand(TurretSubsystem turretSubsystem, VisionSubsystem visionSubsystem) {
        this.turretSubsystem = turretSubsystem;
        this.visionSubsystem = visionSubsystem;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        double targetAngle = visionSubsystem.calculateOdometryTurretAngle();
        turretSubsystem.setAngle(targetAngle);
    }

    /**
     * Continuous command — runs indefinitely while set as default command.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
