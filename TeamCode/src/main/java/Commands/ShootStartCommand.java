package Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import Subsystems.TurretShooterSubsystem;

/**
 * ShootStartCommand — Starts the turret shooter flywheel using closed-loop PID control.
 */
public class ShootStartCommand extends InstantCommand {

    public ShootStartCommand(TurretShooterSubsystem shooterSubsystem) {
        super(shooterSubsystem::spinUpShort, shooterSubsystem);
    }
}
