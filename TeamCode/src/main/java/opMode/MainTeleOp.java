package opMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import Commands.AutoEjectCommand;
import Commands.DefaultDriveCommand;
import Commands.IntakeInCommand;
import Commands.IntakeOutCommand;
import Commands.ShootStartCommand;
import Commands.ShootStopCommand;

import Subsystems.DriveSubsystem;
import Subsystems.FlipperSubsystem;
import Subsystems.IntakeSubsystem;
import Subsystems.TurretShooterSubsystem;

/**
 * MainTeleOp — primary TeleOp OpMode using the FTCLib Command-based framework.
 *
 * <h3>Control Mapping (Gamepad 1)</h3>
 * <table>
 *   <tr><th>Input</th><th>Action</th></tr>
 *   <tr><td>Left Stick Y</td><td>Drive forward / backward</td></tr>
 *   <tr><td>Left Stick X</td><td>Strafe left / right</td></tr>
 *   <tr><td>Right Stick X</td><td>Rotate</td></tr>
 *   <tr><td>Left Trigger (analog)</td><td>Intake IN + Transfer (auto-stop & auto-eject)</td></tr>
 *   <tr><td>Left Bumper</td><td>Intake OUT + Transfer (manual reverse)</td></tr>
 *   <tr><td>X Button</td><td>Manual Auto-Eject / Purge sequence</td></tr>
 *   <tr><td>Right Trigger (analog)</td><td>Turret Shooter START (PID control)</td></tr>
 *   <tr><td>Right Bumper</td><td>Turret Shooter STOP</td></tr>
 * </table>
 *
 * <h3>Autonomous Background Logic</h3>
 * <ul>
 *   <li>{@link IntakeSubsystem}: Monitors 4 color sensors along transfer path. Auto-stops when full, auto-ejects illegal (BLUE) balls.</li>
 *   <li>{@link FlipperSubsystem}: Continuously polls distance sensor and auto-flips detected balls into the turret.</li>
 * </ul>
 *
 * <h3>Performance</h3>
 * REV Control Hub LynxModule bulk caching is enabled for optimized cycle times.
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private DriveSubsystem        driveSubsystem;
    private IntakeSubsystem       intakeSubsystem;
    private FlipperSubsystem      flipperSubsystem;
    private TurretShooterSubsystem turretShooterSubsystem;

    // ── Gamepad wrapper ───────────────────────────────────────────────────────
    private GamepadEx driverGamepad;

    // ── LynxModule bulk reading ───────────────────────────────────────────────
    private List<LynxModule> allHubs;

    @Override
    public void initialize() {
        // ────────────────────────────────────────────────────────────────────
        // 1. Enable LynxModule bulk caching (MANUAL mode — one read per loop)
        // ────────────────────────────────────────────────────────────────────
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ────────────────────────────────────────────────────────────────────
        // 2. Instantiate subsystems
        // ────────────────────────────────────────────────────────────────────
        driveSubsystem        = new DriveSubsystem(hardwareMap, telemetry);
        intakeSubsystem       = new IntakeSubsystem(hardwareMap, telemetry);
        flipperSubsystem      = new FlipperSubsystem(hardwareMap, telemetry);
        turretShooterSubsystem = new TurretShooterSubsystem(hardwareMap, telemetry);

        // ────────────────────────────────────────────────────────────────────
        // 3. Register subsystems with the scheduler
        //    (DriveSubsystem is registered implicitly via setDefaultCommand)
        // ────────────────────────────────────────────────────────────────────
        register(intakeSubsystem, flipperSubsystem, turretShooterSubsystem);

        // ────────────────────────────────────────────────────────────────────
        // 4. Wrap gamepad
        // ────────────────────────────────────────────────────────────────────
        driverGamepad = new GamepadEx(gamepad1);

        // ────────────────────────────────────────────────────────────────────
        // 5. Default commands
        // ────────────────────────────────────────────────────────────────────
        driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
                driveSubsystem,
                () -> -driverGamepad.getLeftY(),   // forward (negate for FTC convention)
                () ->  driverGamepad.getLeftX(),   // strafe
                () ->  driverGamepad.getRightX()   // rotation
        ));

        // ────────────────────────────────────────────────────────────────────
        // 6. Button / trigger bindings
        // ────────────────────────────────────────────────────────────────────
        configureBindings();

        telemetry.addLine(">> Main TeleOp Initialized.");
        telemetry.update();
    }

    /**
     * Map all gamepad inputs to their respective commands.
     */
    private void configureBindings() {
        // ── Intake IN + Transfer — Left Trigger (analog, > 0.05 deadzone) ──
        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        new IntakeInCommand(
                                intakeSubsystem,
                                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                        )
                );

        // ── Intake OUT + Transfer — Left Bumper ───────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new IntakeOutCommand(intakeSubsystem));

        // ── Manual Auto-Eject / Purge — X Button ──────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(new AutoEjectCommand(intakeSubsystem));

        // ── Turret Shooter START — Right Trigger (analog > 0.05) ──────────
        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05)
                .whenActive(new ShootStartCommand(turretShooterSubsystem));

        // ── Turret Shooter STOP — Right Bumper ────────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ShootStopCommand(turretShooterSubsystem));
    }

    /**
     * Called every loop iteration. We clear the LynxModule bulk cache
     * at the top of each loop so that all hardware reads in this cycle
     * use fresh data from a single bulk read.
     */
    @Override
    public void run() {
        // Clear bulk cache — forces a fresh bulk read for this cycle
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Run the FTCLib CommandScheduler (updates subsystems + commands)
        super.run();

        // Push telemetry to driver station
        telemetry.update();
    }
}
