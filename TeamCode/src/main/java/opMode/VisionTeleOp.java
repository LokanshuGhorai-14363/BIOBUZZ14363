package opMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import Commands.AutoEjectCommand;
import Commands.GoalTrackingDriveCommand;
import Commands.IntakeInCommand;
import Commands.IntakeOutCommand;
import Commands.RelocalizeCommand;
import Commands.ScoreCommand;
import Commands.ShootStopCommand;
import Commands.TurretTrackGoalCommand;

import Subsystems.DriveSubsystem;
import Subsystems.IntakeSubsystem;
import Subsystems.TransferSubsystem;
import Subsystems.TurretShooterSubsystem;
import Subsystems.TurretSubsystem;
import Subsystems.VisionSubsystem;
import Subsystems.VisionSubsystem.Alliance;

import util.HiveGoalTracker;

/**
 * VisionTeleOp — Unified Single-Driver Competition TeleOp Base Class.
 *
 * <h3>Design Philosophy: Shortest Cycle Times</h3>
 * Designed for single-driver operation where chassis maneuvering, dynamic intake,
 * continuous goal tracking, and automated scoring occur in parallel without blocking.
 *
 * <h3>Loop Performance Guarantees</h3>
 * <ul>
 *   <li>Manual LynxModule bulk caching: exactly one bulk read per Hub per loop.</li>
 *   <li>{@link HiveGoalTracker} integrated via {@link VisionSubsystem}: computes goal
 *       vectors, distance, hysteresis, and heading error once per loop; all subsystems
 *       and commands read from cache.</li>
 *   <li>100% non-blocking command execution.</li>
 * </ul>
 *
 * <h3>Single-Driver Gamepad 1 Mapping</h3>
 * <ul>
 *   <li><b>Left Stick</b>: Field-centric translation (forward / strafe).</li>
 *   <li><b>Right Stick X</b>: Manual chassis rotation (overrides auto-aim).</li>
 *   <li><b>Left Trigger</b>: Analog proportional Intake IN + Transfer.</li>
 *   <li><b>Left Bumper</b>: Intake OUT / Reverse (full power, held).</li>
 *   <li><b>Right Trigger</b>: Shooter Flywheel ON (dynamic distance-scaled velocity, latched).</li>
 *   <li><b>Right Bumper</b>: Shooter Flywheel OFF.</li>
 *   <li><b>A Button</b>: Ultra-fast Score Macro (spin-up → feed → clear).</li>
 *   <li><b>B Button</b>: Toggle chassis auto-aim ON / OFF.</li>
 *   <li><b>X Button</b>: Auto-eject / Purge (timed 1-second reverse pulse).</li>
 *   <li><b>Y Button</b>: Reset field-centric forward heading.</li>
 *   <li><b>D-Pad Up</b>: Relocalize to top-left coordinate tile.</li>
 *   <li><b>D-Pad Down</b>: Toggle dynamic velocity scaling ON / OFF.</li>
 * </ul>
 */
public abstract class VisionTeleOp extends CommandOpMode {

    protected DriveSubsystem         driveSubsystem;
    protected IntakeSubsystem        intakeSubsystem;
    protected TransferSubsystem      transferSubsystem;
    protected TurretShooterSubsystem turretShooterSubsystem;
    protected TurretSubsystem        turretSubsystem;
    protected VisionSubsystem        visionSubsystem;

    protected HiveGoalTracker  goalTracker;
    protected GamepadEx        driverGamepad;
    protected List<LynxModule> allHubs;
    protected final ElapsedTime loopTimer = new ElapsedTime();
    protected final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_INTERVAL_MS = 10.0; // 50 Hz Driver Station refresh rate (fast telemetry)

    private final Alliance alliance;
    private boolean autoAimEnabled = false; // Disabled by default so chassis won't fight manual driver rotation
    private boolean dynamicVelocityEnabled = true;

    public VisionTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        // Fast telemetry update transmission interval (20ms = 50Hz)
        telemetry.setMsTransmissionInterval(20);

        // 1. Manual LynxModule bulk caching (one bulk read per loop)
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. Instantiate subsystems
        driveSubsystem         = new DriveSubsystem(hardwareMap, telemetry);
        intakeSubsystem        = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem      = new TransferSubsystem(hardwareMap, telemetry);
        turretShooterSubsystem = new TurretShooterSubsystem(hardwareMap, telemetry);
        turretSubsystem        = new TurretSubsystem(hardwareMap, telemetry);
        visionSubsystem        = new VisionSubsystem(hardwareMap, telemetry, turretSubsystem, driveSubsystem);

        // 3. Register subsystems with FTCLib CommandScheduler
        register(driveSubsystem, intakeSubsystem, transferSubsystem,
                 turretShooterSubsystem, turretSubsystem, visionSubsystem);

        // 4. Configure Alliance and starting pose
        visionSubsystem.setAlliance(alliance);
        goalTracker = visionSubsystem.getGoalTracker();
        Pose startPose = visionSubsystem.getStartPose(alliance);
        driveSubsystem.setPose(startPose);
        goalTracker.update(startPose);

        // 5. Gamepad setup
        driverGamepad = new GamepadEx(gamepad1);

        // 6. Default Commands (non-blocking, running in background every loop)
        configureDefaultCommands();

        // 7. Button & Trigger bindings
        configureBindings();

        telemetry.addLine(">> Vision TeleOp Initialized (Single-Driver Mode)");
        telemetry.addData(">> Alliance", alliance);
        telemetry.addData(">> Auto-Aim", autoAimEnabled ? "ACTIVE" : "DISABLED");
        telemetry.addData(">> Start Pose", String.format("(%.1f\", %.1f\", %.1f°)",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading())));
        telemetry.update();
    }

    private void configureDefaultCommands() {
        // Default Drive: Field-centric with PID auto-aim toward active HIVE goal
        driveSubsystem.setDefaultCommand(new GoalTrackingDriveCommand(
                driveSubsystem,
                goalTracker,
                () -> -driverGamepad.getLeftY(),
                () ->  driverGamepad.getLeftX(),
                () ->  driverGamepad.getRightX(),
                () ->  autoAimEnabled
        ));

        // Default Turret: Continuous tracking of active HIVE goal via Pinpoint odometry
        turretSubsystem.setDefaultCommand(new TurretTrackGoalCommand(
                turretSubsystem,
                visionSubsystem
        ));
    }

    private void configureBindings() {
        // ── Intake IN + Transfer (Proportional analog on Left Trigger) ───────
        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05)
                .whileActiveContinuous(
                        new IntakeInCommand(
                                intakeSubsystem,
                                transferSubsystem,
                                () -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
                        )
                );

        // ── Intake OUT + Transfer (Full reverse on Left Bumper) ──────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new IntakeOutCommand(intakeSubsystem, transferSubsystem));

        // ── A Button: Ultra-fast Score Macro (1-touch automated scoring) ──────
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(new ScoreCommand(intakeSubsystem, transferSubsystem, turretShooterSubsystem, goalTracker));

        // ── Right Trigger: Spin Up Shooter Flywheel & Feed elements while held ─
        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
                       || gamepad1.right_trigger > 0.05)
                .whileActiveContinuous(() -> {
                    double triggerVal = Math.max(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), gamepad1.right_trigger);

                    // Guarantee shooter flywheel spins up when attempting to shoot
                    if (!turretShooterSubsystem.isSpinning()) {
                        spinUpFlywheelGuaranteed();
                    }

                    double feedPower = (triggerVal > 0.05) ? Math.max(0.75, triggerVal) : 1.0;
                    transferSubsystem.spinProportional(feedPower);
                    intakeSubsystem.spinIn(feedPower);
                })
                .whenInactive(() -> {
                    transferSubsystem.stop();
                    intakeSubsystem.stop();
                });

        // ── Shooter Flywheel OFF (Right Bumper) ──────────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ShootStopCommand(turretShooterSubsystem));

        // ── Toggle Auto-Aim (B Button) ──────────────────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(() -> {
                    autoAimEnabled = !autoAimEnabled;
                    if (!autoAimEnabled) {
                        goalTracker.resetHeadingPID();
                    }
                });

        // ── Auto-Eject / Purge (X Button: 1-second reverse pulse) ────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(new AutoEjectCommand(intakeSubsystem));

        // ── Reset Field-Centric Heading (Y Button) ──────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(driveSubsystem::resetHeading);

        // ── Relocalize to Top-Left Square (D-Pad Up) ────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new RelocalizeCommand(driveSubsystem, visionSubsystem, telemetry));

        // ── Toggle Dynamic Velocity Scaling (D-Pad Down) ────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> dynamicVelocityEnabled = !dynamicVelocityEnabled);
    }

    public void toggleFlywheel() {
        if (turretShooterSubsystem.isSpinning()) {
            turretShooterSubsystem.stop();
        } else {
            spinUpFlywheelGuaranteed();
        }
    }

    private void spinUpFlywheelGuaranteed() {
        double calcVel = (dynamicVelocityEnabled && goalTracker != null) ? goalTracker.getSuggestedLaunchVelocity() : 0.0;
        double targetVelocity = (calcVel < TurretShooterSubsystem.outtakeVelocityShort || Double.isNaN(calcVel))
                ? TurretShooterSubsystem.outtakeVelocityShort
                : calcVel;
        turretShooterSubsystem.setTargetVelocity(targetVelocity);
    }

    @Override
    public void run() {
        loopTimer.reset();

        // 1. Clear LynxModule bulk cache at loop start for fresh single-read
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // 2. Dynamically update flywheel velocity while spinning if distance auto-scale is ON
        if (turretShooterSubsystem.isSpinning() && dynamicVelocityEnabled && goalTracker != null) {
            double dynamicVel = goalTracker.getSuggestedLaunchVelocity();
            if (dynamicVel >= TurretShooterSubsystem.outtakeVelocityShort) {
                turretShooterSubsystem.setTargetVelocity(dynamicVel);
            }
        }

        // 3. Run CommandScheduler (executes all subsystem periodic() and active commands)
        super.run();

        // 4. Telemetry diagnostics (Throttled to 10Hz to prevent string allocation & DS I/O loop lag)
        if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
            telemetryTimer.reset();
            emitTelemetry(driveSubsystem.getPose());
            telemetry.update();
        }
    }

    private void emitTelemetry(Pose robotPose) {
        telemetry.addLine("──── TELEOP STATUS ────");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Goal", String.format("%s (%.0f, %.0f) | %.1fin",
                goalTracker.isTargetingFar() ? "FAR" : "NEAR",
                goalTracker.getTargetX(), goalTracker.getTargetY(),
                goalTracker.getDistanceInches()));
        telemetry.addData("Auto-Aim", autoAimEnabled ? "ON" : "OFF");
        telemetry.addData("Hdg Err", String.format("%.1f°", goalTracker.getHeadingErrorDeg()));
        telemetry.addData("Shooter", String.format("%s (%.0f / %.0f RPM)",
                turretShooterSubsystem.getState(),
                turretShooterSubsystem.getCurrentRPM(),
                turretShooterSubsystem.getTargetRPM()));
        telemetry.addData("Pose", String.format("(%.1f, %.1f) %.1f°",
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));
        telemetry.addData("Loop (ms)", String.format("%.1f", loopTimer.milliseconds()));
        telemetry.addLine("───────────────────────");
    }

    public Alliance getAlliance() {
        return alliance;
    }
}
