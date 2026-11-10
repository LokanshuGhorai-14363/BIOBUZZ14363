package opMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import Commands.GoalTrackingDriveCommand;
import Commands.IntakeOutCommand;
import Commands.RelocalizeCommand;
import Commands.TurretTrackGoalCommand;

import Subsystems.DriveSubsystem;
import Subsystems.FlowerSubsystem;
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
 *   <li><b>Left Trigger</b>: Intake IN (analog proportional).</li>
 *   <li><b>Right Trigger</b>: Intake + Transfer (analog proportional).</li>
 *   <li><b>B Button</b>: Intake + Transfer (full power, held).</li>
 *   <li><b>Left Bumper</b>: Eject / Reverse Intake + Transfer (held).</li>
 *   <li><b>Right Bumper</b>: Shooter Flywheel ON / OFF toggle.</li>
 *   <li><b>D-Pad Up</b>: Relocalize at Gardens (Red (0,0), Blue (144,144)).</li>
 *   <li><b>D-Pad Down</b>: IMU Reset / Field-centric heading reset.</li>
 * </ul>
 */
public abstract class VisionTeleOp extends CommandOpMode {

    protected DriveSubsystem         driveSubsystem;
    protected IntakeSubsystem        intakeSubsystem;
    protected TransferSubsystem      transferSubsystem;
    protected TurretShooterSubsystem turretShooterSubsystem;
    protected TurretSubsystem        turretSubsystem;
    protected VisionSubsystem        visionSubsystem;
    protected FlowerSubsystem        flowerSubsystem;

    protected HiveGoalTracker  goalTracker;
    protected GamepadEx        driverGamepad;
    protected GamepadEx        coDriverGamepad;
    protected List<LynxModule> allHubs;
    protected final ElapsedTime loopTimer = new ElapsedTime();
    protected final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_INTERVAL_MS = 10.0; // 50 Hz Driver Station refresh rate (fast telemetry)

    private final Alliance alliance;
    private boolean autoAimEnabled = false; // Disabled by default so chassis won't fight manual driver rotation
    private boolean dynamicVelocityEnabled = true;

    // Gamepad 2 (Co-Driver) Manual Turret Control State
    private boolean manualTurretMode = false; // false = Odometry Auto-Tracking, true = Manual (Gamepad 2 Left Stick X)

    public VisionTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        // Clear subsystems/commands left in the singleton scheduler by a previous OpMode that
        // crashed (FTCLib only resets it when an OpMode ends normally). Stale subsystems from
        // those runs caused NPEs in periodic() at START (e.g. IntakeSubsystem line 63).
        CommandScheduler.getInstance().reset();

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
        flowerSubsystem        = new FlowerSubsystem(hardwareMap, telemetry);

        // 3. Register subsystems with FTCLib CommandScheduler
        register(driveSubsystem, intakeSubsystem, transferSubsystem,
                 turretShooterSubsystem, turretSubsystem, visionSubsystem, flowerSubsystem);

        // 4. Configure Alliance and starting pose
        visionSubsystem.setAlliance(alliance);
        driveSubsystem.setAlliance(alliance);
        goalTracker = visionSubsystem.getGoalTracker();
        Pose startPose = visionSubsystem.getStartPose(alliance);
        driveSubsystem.setPose(startPose);
        goalTracker.update(startPose);

        // 5. Gamepad setup
        driverGamepad   = new GamepadEx(gamepad1);
        coDriverGamepad = new GamepadEx(gamepad2);

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
                // Same stick mapping as BasicFieldCentricTeleOp (raw gamepad, +fwd, +left, +CCW)
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () ->  autoAimEnabled
        ));

        // Default Turret: Continuous odometry goal tracking OR manual Co-Driver stick control
        turretSubsystem.setDefaultCommand(new TurretTrackGoalCommand(
                turretSubsystem,
                visionSubsystem,
                () -> manualTurretMode,
                () -> coDriverGamepad.getLeftX()
        ));
    }

    private void configureBindings() {
        // ── Left Trigger: Just Intake ─────────────────────────────────────────
        new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05
                       || gamepad1.left_trigger > 0.05)
                .whileActiveContinuous(() -> {
                    double triggerVal = Math.max(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), gamepad1.left_trigger);
                    double power = (triggerVal > 0.05) ? Math.max(0.75, triggerVal) : 1.0;
                    intakeSubsystem.spinIn(power);
                })
                .whenInactive(() -> intakeSubsystem.stop());

        // ── Right Trigger: Intake + Transfer while held, toggle Flower Servo (0° <-> 90°) on press ─
        Trigger rightTrigger = new Trigger(() -> driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05
                                             || gamepad1.right_trigger > 0.05);

        rightTrigger.whenActive(flowerSubsystem::toggle);

        rightTrigger.whileActiveContinuous(() -> {
            double triggerVal = Math.max(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), gamepad1.right_trigger);
            double power = (triggerVal > 0.05) ? Math.max(0.75, triggerVal) : 1.0;
            intakeSubsystem.spinIn(power);
            transferSubsystem.spinProportional(power);
        })
        .whenInactive(() -> {
            intakeSubsystem.stop();
            transferSubsystem.stop();
        });

        // ── B Button: Intake + Transfer ───────────────────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whileHeld(() -> {
                    intakeSubsystem.spinIn(1.0);
                    transferSubsystem.spinProportional(1.0);
                })
                .whenReleased(() -> {
                    intakeSubsystem.stop();
                    transferSubsystem.stop();
                });

        // ── Left Bumper: Eject (Intake OUT + Transfer Reverse, no timer) ──────
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new IntakeOutCommand(intakeSubsystem, transferSubsystem));

        // ── Right Bumper: Flywheel ON / OFF (Toggle) ─────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(this::toggleFlywheel);

        // ── D-Pad Up: Relocalize at Gardens (Red (0,0), Blue (144,144)) ──────
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new RelocalizeCommand(driveSubsystem, visionSubsystem, telemetry));

        // ── D-Pad Down: IMU Reset / Reset Heading ────────────────────────────
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(driveSubsystem::resetHeading);

        // ── Gamepad 2 (Co-Driver): Turret Mode & Flywheel Velocity Controls ──
        // A Button: Toggle between Odometry Goal Tracking and Manual Stick Control
        new GamepadButton(coDriverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> manualTurretMode = !manualTurretMode);

        // Right Bumper: Increase shooter flywheel velocity target (+100 ticks/sec ~ +215 RPM)
        new GamepadButton(coDriverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {
                    double currentVel = turretShooterSubsystem.getTargetVelocity();
                    double newVel = (currentVel <= 0) ? TurretShooterSubsystem.outtakeVelocityShort : currentVel + 100.0;
                    turretShooterSubsystem.setTargetVelocity(newVel);
                });

        // Left Bumper: Decrease shooter flywheel velocity target (-100 ticks/sec ~ -215 RPM)
        new GamepadButton(coDriverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {
                    double currentVel = turretShooterSubsystem.getTargetVelocity();
                    double newVel = Math.max(0.0, currentVel - 100.0);
                    turretShooterSubsystem.setTargetVelocity(newVel);
                });
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

        // 2. Fetch fresh robot pose from driveSubsystem at the top of main periodic loop
        Pose currentPose = driveSubsystem.getPose();
        if (goalTracker != null) {
            goalTracker.update(currentPose);
        }

        // 3. Dynamically update flywheel velocity while spinning if distance auto-scale is ON
        if (turretShooterSubsystem.isSpinning() && dynamicVelocityEnabled && goalTracker != null) {
            double dynamicVel = goalTracker.getSuggestedLaunchVelocity();
            if (dynamicVel >= TurretShooterSubsystem.outtakeVelocityShort) {
                turretShooterSubsystem.setTargetVelocity(dynamicVel);
            }
        }

        // 4. Run CommandScheduler (executes all subsystem periodic() and active commands)
        super.run();

        // 5. Telemetry diagnostics (Throttled to 10Hz to prevent string allocation & DS I/O loop lag)
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
        telemetry.addData("Turret Mode", manualTurretMode
                ? "MANUAL (Gamepad 2 Stick X Analog @ Max 250°/s)"
                : "AUTO-TRACKING (Odometry)");
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
