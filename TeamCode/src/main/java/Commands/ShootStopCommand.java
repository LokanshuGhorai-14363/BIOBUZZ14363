package Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import Subsystems.TurretShooterSubsystem;

/**
 * ShootStopCommand — Stops the turret shooter motors.
 */
public class ShootStopCommand extends InstantCommand {

    public ShootStopCommand(TurretShooterSubsystem shooterSubsystem) {
        super(shooterSubsystem::stop, shooterSubsystem);
    }
}
