package opMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import Commands.AutoEjectCommand;
import Commands.DefaultDriveCommand;
import Commands.FieldCentricDriveCommand;
import Commands.IntakeInCommand;
import Commands.IntakeOutCommand;
import Commands.ShootStartCommand;
import Commands.ShootStopCommand;

import Subsystems.DriveSubsystem;
import Subsystems.IntakeSubsystem;
import Subsystems.TransferSubsystem;
import Subsystems.TurretShooterSubsystem;
import Subsystems.TurretSubsystem;
import Subsystems.VisionSubsystem;
import Subsystems.VisionSubsystem.Alliance;

/**
 * MainTeleOp — primary TeleOp OpMode using the FTCLib Command-based framework.
 * Features automated Pinpoint odometry turret tracking to alliance goals while driving.
 */
@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private DriveSubsystem        driveSubsystem;
    private IntakeSubsystem       intakeSubsystem;
    private TransferSubsystem     transferSubsystem;
    private TurretShooterSubsystem turretShooterSubsystem;
    private TurretSubsystem       turretSubsystem;
    private VisionSubsystem       visionSubsystem;

    // ── Gamepad wrapper ───────────────────────────────────────────────────────
    private GamepadEx driverGamepad;

    // ── LynxModule bulk reading ───────────────────────────────────────────────
    private List<LynxModule> allHubs;

    // Default to Red Alliance for Main TeleOp (change to BLUE if on Blue alliance)
    private Alliance alliance = Alliance.RED;

    // Field-centric driving is enabled by default
    private boolean fieldCentric = true;

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
        driveSubsystem         = new DriveSubsystem(hardwareMap, telemetry);
        intakeSubsystem        = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem      = new TransferSubsystem(hardwareMap, telemetry);
        turretShooterSubsystem = new TurretShooterSubsystem(hardwareMap, telemetry);
        turretSubsystem        = new TurretSubsystem(hardwareMap, telemetry);
        visionSubsystem        = new VisionSubsystem(hardwareMap, telemetry, turretSubsystem, driveSubsystem);

        // ────────────────────────────────────────────────────────────────────
        // 3. Register subsystems with the scheduler
        // ────────────────────────────────────────────────────────────────────
        register(intakeSubsystem, transferSubsystem, turretShooterSubsystem, turretSubsystem, visionSubsystem);

        // Configure alliance and initial robot field pose
        visionSubsystem.setAlliance(alliance);
        Pose startPose = visionSubsystem.getStartPose(alliance);
        driveSubsystem.setPose(startPose);

        // ────────────────────────────────────────────────────────────────────
        // 4. Wrap gamepad
        // ────────────────────────────────────────────────────────────────────
        driverGamepad = new GamepadEx(gamepad1);

        // ────────────────────────────────────────────────────────────────────
        // 5. Default commands
        // ────────────────────────────────────────────────────────────────────
        // Default Drive Command — Field-Centric using Pinpoint heading
        driveSubsystem.setDefaultCommand(new FieldCentricDriveCommand(
                driveSubsystem,
                () -> -driverGamepad.getLeftY(),   // forward (field-relative)
                () ->  driverGamepad.getLeftX(),   // strafe  (field-relative)
                () ->  driverGamepad.getRightX()   // rotation (always robot-relative)
        ));

        // Default Turret Command — Automatically tracks alliance goal using Pinpoint Odometry
        turretSubsystem.setDefaultCommand(new RunCommand(
                () -> {
                    double targetAngle = visionSubsystem.calculateOdometryTurretAngle();
                    turretSubsystem.setAngle(targetAngle);
                },
                turretSubsystem
        ));

        // ────────────────────────────────────────────────────────────────────
        // 6. Button / trigger bindings
        // ────────────────────────────────────────────────────────────────────
        configureBindings();

        telemetry.addLine(">> Main TeleOp Initialized — FIELD-CENTRIC DRIVE");
        telemetry.addData(">> Alliance", alliance);
        telemetry.addData(">> Reset Heading", "Press Y to re-zero forward direction");
        telemetry.update();
    }

    /**
     * Map all gamepad inputs to their respective commands.
     */
    private void configureBindings() {
        // ── Intake IN + Transfer — Left Trigger (proportional analog, > 0.05 deadzone) ──
        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        new IntakeInCommand(
                                intakeSubsystem,
                                transferSubsystem,
                                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                        )
                );

        // ── Intake OUT + Transfer — Left Bumper (full reverse) ────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new IntakeOutCommand(intakeSubsystem, transferSubsystem));

        // ── Manual Auto-Eject / Purge — X Button ──────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(new AutoEjectCommand(intakeSubsystem));

        // ── Manual Transfer — A Button ────────────────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(transferSubsystem::transfer);

        // ── Reset Field-Centric Heading — Y Button ────────────────────────
        // Re-zeros which direction counts as "forward" on the joystick.
        // Driver should face the robot away from them and press Y.
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(driveSubsystem::resetHeading);

        // ── Turret Shooter START / Latch ON — Right Trigger (press to latch) ──
        // A single pull of the right trigger latches the shooter ON at the
        // default short-range speed. It stays spinning until the driver
        // explicitly presses Right Bumper to stop.
        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
                       || gamepad1.right_trigger > 0.05)
                .whenActive(
                        new ShootStartCommand(turretShooterSubsystem)
                );

        // ── Turret Shooter STOP / Latch OFF — Right Bumper ────────────────
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
