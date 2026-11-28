package opMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.telemetry.Plugin;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.Locale;

import Commands.GoalTrackingDriveCommand;
import Commands.TurretTrackGoalCommand;
import Subsystems.DriveSubsystem;
import Subsystems.FlowerSubsystem;
import Subsystems.IntakeSubsystem;
import Subsystems.TransferSubsystem;
import Subsystems.TurretShooterSubsystem;
import Subsystems.TurretSubsystem;
import Subsystems.VisionSubsystem;
import Subsystems.VisionSubsystem.Alliance;
import util.BallisticsCalculator;
import util.BallisticsConfig;
import util.HiveGoalTracker;
import util.MultipleTelemetry;
import util.PoseStorage;

/**
 * <h1>ShooterTuningTeleOp</h1>
 *
 * Dedicated Competition-grade TeleOp OpMode for live empirical calibration of the
 * unified projectile ballistics model for the BIOBUZZ single-flywheel shooter mechanism.
 *
 * <h2>Unified Ballistics Model</h2>
 * Fires both Pollen and Nectar game elements without active color identification using
 * a unified compromise RPM calculation:
 * <pre>
 *   Target RPM = (Base RPM * COMPROMISE_MULTIPLIER) + COMPROMISE_OFFSET_RPM
 * </pre>
 * where {@code Base RPM} is derived analytically from 2D projectile kinematics:
 * <pre>
 *   v₀ = (x / cos(θ)) * sqrt( g / (2 * (x * tan(θ) - y)) )
 *   v_wheel = v₀ / LAUNCH_EFFICIENCY
 *   Base RPM = (v_wheel * 60) / WHEEL_CIRCUMFERENCE_CM
 * </pre>
 *
 * <h2>Controls &amp; Button Mappings</h2>
 * <ul>
 *   <li><b>Left Stick (Y/X)</b>: Field-Centric Translation (Forward / Strafe)</li>
 *   <li><b>Right Stick X</b>: Manual Chassis Rotation</li>
 *   <li><b>Right Bumper</b>: Toggle Flywheel ON / OFF (spins up to calculated Target RPM)</li>
 *   <li><b>B Button (Held)</b>: Fire element (Deploys Flower mech + Feeds intake and transfer)</li>
 *   <li><b>X Button</b>: Toggle between Analytical Odometry Mode and Fixed-Distance Mode</li>
 *   <li><b>Y Button</b>: Cycle Fixed Distance (36" → 60" → 84" → 108")</li>
 *   <li><b>D-Pad UP / DOWN</b>: Adjust COMPROMISE_MULTIPLIER (±0.005 coarse, ±0.001 fine with Left Bumper held)</li>
 *   <li><b>D-Pad RIGHT / LEFT</b>: Adjust COMPROMISE_OFFSET_RPM (±10 RPM coarse, ±2 RPM fine with Left Bumper held)</li>
 *   <li><b>Back / Share Button</b>: Reset Multiplier &amp; Offset to baseline defaults</li>
 *   <li><b>Left Bumper (Held)</b>: Fine-tuning modifier (hold while pressing D-Pad)</li>
 * </ul>
 *
 * <h2>FTC Dashboard Note</h2>
 * If {@code FtcDashboard} is installed in your project dependencies, uncomment the
 * {@code @Config} annotation above this class to tune these constants live from a laptop browser.
 */
@TeleOp(name = "Shooter Tuning TeleOp", group = "Tuning")
// @com.acmerobotics.dashboard.config.Config // <-- Uncomment if com.acmerobotics.dashboard is added to build.gradle
public class ShooterTuningTeleOp extends CommandOpMode {

    // =========================================================================
    //  Physical Hardware Constants — now read from BallisticsConfig
    // =========================================================================
    //  WHEEL_CIRCUMFERENCE_CM, LAUNCH_ANGLE_DEGREES, TARGET_HEIGHT_INCHES,
    //  TARGET_DEPTH_OFFSET_INCHES, and LAUNCH_EFFICIENCY have been migrated to
    //  util.BallisticsConfig (all in INCHES, not cm).  The fields below are kept
    //  only as convenient UI aliases that sync TO BallisticsConfig each cycle so
    //  both Dashboard pages (this class and BallisticsConfig) stay consistent.
    //
    //  Prefer editing BallisticsConfig directly for new code.
    // =========================================================================

    /**
     * @deprecated Use {@link BallisticsConfig#WHEEL_CIRCUMFERENCE_IN} (inches) instead.
     *             This field exists only for backward-compatible Dashboard display.
     */
    public static double WHEEL_CIRCUMFERENCE_CM = 21.99;

    /**
     * @deprecated Use {@link BallisticsConfig#LAUNCH_ANGLE_DEG} instead.
     */
    public static double LAUNCH_ANGLE_DEGREES = 45.0;

    /**
     * @deprecated Use {@link BallisticsConfig#TARGET_HEIGHT_IN} instead.
     */
    public static double TARGET_HEIGHT_INCHES = 36.0;

    /**
     * @deprecated Use {@link BallisticsConfig#TARGET_DEPTH_OFFSET_IN} instead.
     */
    public static double TARGET_DEPTH_OFFSET_INCHES = 3.0;

    /**
     * @deprecated Use {@link BallisticsConfig#LAUNCH_EFFICIENCY} instead.
     */
    public static double LAUNCH_EFFICIENCY = 0.50;

    // =========================================================================
    //  Tuning Constants & Baseline Defaults
    // =========================================================================

    public static final double DEFAULT_MULTIPLIER = 1.000;
    public static final double DEFAULT_OFFSET_RPM = 0.0;

    /** Tunable unified multiplier. */
    public static double COMPROMISE_MULTIPLIER = DEFAULT_MULTIPLIER;

    /** Tunable unified base offset RPM. */
    public static double COMPROMISE_OFFSET_RPM = DEFAULT_OFFSET_RPM;

    /** Fixed test distances in inches for offline/stationary tuning. */
    public static final double[] FIXED_TEST_DISTANCES_INCHES = { 36.0, 60.0, 84.0, 108.0 };
    private int fixedDistanceIndex = 1; // Default to 60.0"

    // =========================================================================
    //  Operating Modes & Subsystems
    // =========================================================================

    public enum AimMode {
        ANALYTICAL_ODOMETRY,
        MANUAL_FIXED_DISTANCE
    }

    private AimMode aimMode = AimMode.ANALYTICAL_ODOMETRY;

    private DriveSubsystem         driveSubsystem;
    private IntakeSubsystem        intakeSubsystem;
    private TransferSubsystem      transferSubsystem;
    private TurretShooterSubsystem turretShooterSubsystem;
    private TurretSubsystem        turretSubsystem;
    private VisionSubsystem        visionSubsystem;
    private FlowerSubsystem        flowerSubsystem;
    private HiveGoalTracker        goalTracker;

    private GamepadEx driverGamepad;
    private GamepadEx coDriverGamepad;
    private List<LynxModule> allHubs;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_INTERVAL_MS = 20.0; // 50 Hz Driver Station update

    private Alliance currentAlliance = Alliance.RED;
    private boolean flywheelSpinningUserToggle = false;

    // Edge-detection states for gamepad buttons
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;
    private boolean lastButtonX = false;
    private boolean lastButtonY = false;
    private boolean lastButtonBack = false;

    // Cached calculation values
    private double currentDistanceInches = 60.0;
    private double calculatedBaseRPM = 0.0;
    private double calculatedTargetRPM = 0.0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        try {
            this.telemetry = new MultipleTelemetry(
                    this.telemetry,
                    Plugin.INSTANCE.getManager().getWrapper()
            );
        } catch (Throwable ignored) {}

        telemetry.setMsTransmissionInterval(20);

        // 1. Configure LynxModule manual bulk caching for highest loop rate
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. Instantiate all robot subsystems
        driveSubsystem         = new DriveSubsystem(hardwareMap, telemetry);
        intakeSubsystem        = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem      = new TransferSubsystem(hardwareMap, telemetry);
        turretShooterSubsystem = new TurretShooterSubsystem(hardwareMap, telemetry);
        turretSubsystem        = new TurretSubsystem(hardwareMap, telemetry);
        visionSubsystem        = new VisionSubsystem(hardwareMap, telemetry, turretSubsystem, driveSubsystem);
        flowerSubsystem        = new FlowerSubsystem(hardwareMap, telemetry);

        // 3. Register subsystems with FTCLib
        register(driveSubsystem, intakeSubsystem, transferSubsystem,
                 turretShooterSubsystem, turretSubsystem, visionSubsystem, flowerSubsystem);

        // 4. Configure Alliance & starting pose
        visionSubsystem.setAlliance(currentAlliance);
        driveSubsystem.setAlliance(currentAlliance);
        goalTracker = visionSubsystem.getGoalTracker();

        Pose startPose = (PoseStorage.currentPose != null)
                ? PoseStorage.currentPose
                : visionSubsystem.getStartPose(currentAlliance);
        driveSubsystem.setPose(startPose);
        goalTracker.update(startPose);

        // 5. Gamepad setup
        driverGamepad   = new GamepadEx(gamepad1);
        coDriverGamepad = new GamepadEx(gamepad2);

        // 6. Set Default Drive & Turret tracking commands
        driveSubsystem.setDefaultCommand(new GoalTrackingDriveCommand(
                driveSubsystem,
                goalTracker,
                () -> -gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x,
                () -> -gamepad1.right_stick_x,
                () -> false // Disable chassis auto-aim so driver has full freedom of movement
        ));

        turretSubsystem.setDefaultCommand(new TurretTrackGoalCommand(
                turretSubsystem,
                visionSubsystem,
                () -> (aimMode == AimMode.MANUAL_FIXED_DISTANCE), // Keep turret facing forward in fixed mode
                () -> coDriverGamepad.getLeftX()
        ));

        // 7. Bind Firing & Flywheel triggers
        configureFiringBindings();

        telemetry.addLine(">> ShooterTuningTeleOp Initialized!");
        telemetry.addLine(">> Ready for Live Ballistics Calibration.");
        telemetry.update();
    }

    private void configureFiringBindings() {
        // Right Bumper: Toggle Flywheel spinning ON / OFF
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> {
                    flywheelSpinningUserToggle = !flywheelSpinningUserToggle;
                    if (!flywheelSpinningUserToggle) {
                        turretShooterSubsystem.stop();
                    }
                });

        // B Button (Hold to Shoot): Deploy Flower mech + Feed intake and transfer into shooter
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whileHeld(() -> {
                    flowerSubsystem.setAngle90();
                    intakeSubsystem.spinIn(1.0);
                    transferSubsystem.spinProportional(0.75);
                })
                .whenReleased(() -> {
                    flowerSubsystem.setAngle0();
                    intakeSubsystem.stop();
                    transferSubsystem.stop();
                });
    }

    @Override
    public void run() {
        loopTimer.reset();

        try {
            GamepadManager mgr1 = com.bylazar.gamepad.Plugin.INSTANCE.getFirstManager();
            if (Math.abs(mgr1.getLeftStickX()) > 0.02) gamepad1.left_stick_x = (float) mgr1.getLeftStickX();
            if (Math.abs(mgr1.getLeftStickY()) > 0.02) gamepad1.left_stick_y = (float) mgr1.getLeftStickY();
            if (Math.abs(mgr1.getRightStickX()) > 0.02) gamepad1.right_stick_x = (float) mgr1.getRightStickX();
            if (Math.abs(mgr1.getRightStickY()) > 0.02) gamepad1.right_stick_y = (float) mgr1.getRightStickY();
            if (mgr1.getL2() > 0.02) gamepad1.left_trigger = (float) mgr1.getL2();
            if (mgr1.getR2() > 0.02) gamepad1.right_trigger = (float) mgr1.getR2();
            if (mgr1.getL1()) gamepad1.left_bumper = true;
            if (mgr1.getR1()) gamepad1.right_bumper = true;
            if (mgr1.getCross()) gamepad1.a = true;
            if (mgr1.getCircle()) gamepad1.b = true;
            if (mgr1.getSquare()) gamepad1.x = true;
            if (mgr1.getTriangle()) gamepad1.y = true;
            if (mgr1.getDpadUp()) gamepad1.dpad_up = true;
            if (mgr1.getDpadDown()) gamepad1.dpad_down = true;
            if (mgr1.getDpadLeft()) gamepad1.dpad_left = true;
            if (mgr1.getDpadRight()) gamepad1.dpad_right = true;

            GamepadManager mgr2 = com.bylazar.gamepad.Plugin.INSTANCE.getSecondManager();
            if (Math.abs(mgr2.getLeftStickX()) > 0.02) gamepad2.left_stick_x = (float) mgr2.getLeftStickX();
            if (Math.abs(mgr2.getLeftStickY()) > 0.02) gamepad2.left_stick_y = (float) mgr2.getLeftStickY();
            if (Math.abs(mgr2.getRightStickX()) > 0.02) gamepad2.right_stick_x = (float) mgr2.getRightStickX();
            if (Math.abs(mgr2.getRightStickY()) > 0.02) gamepad2.right_stick_y = (float) mgr2.getRightStickY();
            if (mgr2.getL2() > 0.02) gamepad2.left_trigger = (float) mgr2.getL2();
            if (mgr2.getR2() > 0.02) gamepad2.right_trigger = (float) mgr2.getR2();
            if (mgr2.getL1()) gamepad2.left_bumper = true;
            if (mgr2.getR1()) gamepad2.right_bumper = true;
            if (mgr2.getCross()) gamepad2.a = true;
            if (mgr2.getCircle()) gamepad2.b = true;
            if (mgr2.getSquare()) gamepad2.x = true;
            if (mgr2.getTriangle()) gamepad2.y = true;
        } catch (Throwable ignored) {}

        // 1. Clear LynxModule bulk cache at start of loop
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // 2. Refresh odometry pose and goal tracking
        driveSubsystem.periodic();
        Pose currentPose = driveSubsystem.getPose();
        if (goalTracker != null) {
            goalTracker.update(currentPose);
        }

        // 3. Process Live Tuning Gamepad Controls (Rising-edge debounced)
        handleLiveTuningControls();

        // 4. Calculate Distance, Base RPM, and Final Target RPM
        computeBallistics(currentPose);

        // 5. Update Flywheel Target Velocity if toggled active
        if (flywheelSpinningUserToggle) {
            turretShooterSubsystem.setTargetRPM(calculatedTargetRPM);
        }

        // 6. Run active command scheduler
        super.run();

        // 7. Throttled Telemetry output
        if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
            telemetryTimer.reset();
            emitDiagnostics(currentPose);
            telemetry.update();
        }
    }

    // =========================================================================
    //  Live Tuning Gamepad Inputs
    // =========================================================================

    private void handleLiveTuningControls() {
        boolean lbModifier = gamepad1.left_bumper || gamepad2.left_bumper;

        // Step sizes based on modifier
        double multStep = lbModifier ? 0.001 : 0.005;
        double offsetStep = lbModifier ? 2.0 : 10.0;

        // D-Pad UP: Increment COMPROMISE_MULTIPLIER
        boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        if (dpadUp && !lastDpadUp) {
            COMPROMISE_MULTIPLIER += multStep;
        }
        lastDpadUp = dpadUp;

        // D-Pad DOWN: Decrement COMPROMISE_MULTIPLIER
        boolean dpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
        if (dpadDown && !lastDpadDown) {
            COMPROMISE_MULTIPLIER = Math.max(0.1, COMPROMISE_MULTIPLIER - multStep);
        }
        lastDpadDown = dpadDown;

        // D-Pad RIGHT: Increment COMPROMISE_OFFSET_RPM
        boolean dpadRight = gamepad1.dpad_right || gamepad2.dpad_right;
        if (dpadRight && !lastDpadRight) {
            COMPROMISE_OFFSET_RPM += offsetStep;
        }
        lastDpadRight = dpadRight;

        // D-Pad LEFT: Decrement COMPROMISE_OFFSET_RPM
        boolean dpadLeft = gamepad1.dpad_left || gamepad2.dpad_left;
        if (dpadLeft && !lastDpadLeft) {
            COMPROMISE_OFFSET_RPM -= offsetStep;
        }
        lastDpadLeft = dpadLeft;

        // X Button: Toggle between Analytical Odometry and Fixed-Distance Mode
        boolean buttonX = gamepad1.x || gamepad2.x;
        if (buttonX && !lastButtonX) {
            aimMode = (aimMode == AimMode.ANALYTICAL_ODOMETRY)
                    ? AimMode.MANUAL_FIXED_DISTANCE
                    : AimMode.ANALYTICAL_ODOMETRY;
        }
        lastButtonX = buttonX;

        // Y Button: Step through Fixed Distance presets
        boolean buttonY = gamepad1.y || gamepad2.y;
        if (buttonY && !lastButtonY) {
            fixedDistanceIndex = (fixedDistanceIndex + 1) % FIXED_TEST_DISTANCES_INCHES.length;
        }
        lastButtonY = buttonY;

        // Back / Share: Reset constants to baseline defaults
        boolean buttonBack = gamepad1.back || gamepad2.back;
        if (buttonBack && !lastButtonBack) {
            COMPROMISE_MULTIPLIER = DEFAULT_MULTIPLIER;
            COMPROMISE_OFFSET_RPM = DEFAULT_OFFSET_RPM;
        }
        lastButtonBack = buttonBack;
    }

    // =========================================================================
    //  Kinematic Ballistics Calculation
    // =========================================================================

    private void computeBallistics(Pose currentPose) {
        // ── 0. Sync legacy Dashboard aliases → BallisticsConfig ──────────────
        //    (So that edits made via THIS class's Dashboard fields propagate
        //     into the shared model for the next cycle.)
        BallisticsConfig.LAUNCH_ANGLE_DEG       = LAUNCH_ANGLE_DEGREES;
        BallisticsConfig.TARGET_HEIGHT_IN        = TARGET_HEIGHT_INCHES;
        BallisticsConfig.TARGET_DEPTH_OFFSET_IN  = TARGET_DEPTH_OFFSET_INCHES;
        BallisticsConfig.LAUNCH_EFFICIENCY       = LAUNCH_EFFICIENCY;
        BallisticsConfig.WHEEL_CIRCUMFERENCE_IN  = WHEEL_CIRCUMFERENCE_CM / 2.54; // cm → in
        BallisticsConfig.TUNING_RPM_MULTIPLIER   = COMPROMISE_MULTIPLIER;
        BallisticsConfig.TUNING_RPM_OFFSET       = COMPROMISE_OFFSET_RPM;

        // ── 1. Determine horizontal distance ─────────────────────────────────
        if (aimMode == AimMode.ANALYTICAL_ODOMETRY) {
            currentDistanceInches = (goalTracker != null) ? goalTracker.getDistanceInches() : 60.0;
        } else {
            currentDistanceInches = FIXED_TEST_DISTANCES_INCHES[fixedDistanceIndex];
        }

        // ── 2. Delegate to centralized BallisticsCalculator ──────────────────
        calculatedTargetRPM = BallisticsCalculator.computeTargetRPM(currentDistanceInches);

        // Also compute the "base RPM" (before tuning multiplier/offset) for
        // telemetry by temporarily neutralizing the tuning knobs.
        double savedMult   = BallisticsConfig.TUNING_RPM_MULTIPLIER;
        double savedOffset = BallisticsConfig.TUNING_RPM_OFFSET;
        BallisticsConfig.TUNING_RPM_MULTIPLIER = 1.0;
        BallisticsConfig.TUNING_RPM_OFFSET     = 0.0;
        calculatedBaseRPM = BallisticsCalculator.computeTargetRPM(currentDistanceInches);
        BallisticsConfig.TUNING_RPM_MULTIPLIER = savedMult;
        BallisticsConfig.TUNING_RPM_OFFSET     = savedOffset;

        // ── 3. Clamp to motor physical limits ────────────────────────────────
        calculatedTargetRPM = Math.max(0.0, Math.min(TurretShooterSubsystem.MOTOR_MAX_RPM, calculatedTargetRPM));
    }

    /**
     * Analytical Projectile Kinematics formula — now delegates to
     * {@link BallisticsCalculator#computeTargetRPM(double)}.
     * <p>
     * Kept for backward compatibility; new code should call
     * {@code BallisticsCalculator.computeTargetRPM()} directly.
     *
     * @param horizontalDistanceInches Horizontal distance to target lip in inches.
     * @return Ideal base flywheel RPM (before empirical multipliers).
     */
    public static double calculateBaseRPM(double horizontalDistanceInches) {
        // Temporarily neutralize tuning multiplier/offset to get "base" RPM
        double savedMult   = BallisticsConfig.TUNING_RPM_MULTIPLIER;
        double savedOffset = BallisticsConfig.TUNING_RPM_OFFSET;
        BallisticsConfig.TUNING_RPM_MULTIPLIER = 1.0;
        BallisticsConfig.TUNING_RPM_OFFSET     = 0.0;
        double baseRPM = BallisticsCalculator.computeTargetRPM(horizontalDistanceInches);
        BallisticsConfig.TUNING_RPM_MULTIPLIER = savedMult;
        BallisticsConfig.TUNING_RPM_OFFSET     = savedOffset;
        return baseRPM;
    }

    // =========================================================================
    //  Telemetry & Diagnostics
    // =========================================================================

    private void emitDiagnostics(Pose robotPose) {
        double currentRPM = turretShooterSubsystem.getCurrentRPM();
        double rpmError = calculatedTargetRPM - currentRPM;
        boolean atSpeed = turretShooterSubsystem.isAtTargetSpeed() && flywheelSpinningUserToggle;

        telemetry.addLine("════════ SHOOTER TUNING TELEOP ════════");

        // Firing readiness banner
        if (!flywheelSpinningUserToggle) {
            telemetry.addLine("STATUS: [FLYWHEEL OFF] (Press RIGHT BUMPER)");
        } else if (atSpeed) {
            telemetry.addLine("STATUS: >>> READY TO FIRE (AT SPEED) <<<");
        } else {
            telemetry.addLine("STATUS: [SPINNING UP / ADJUSTING VELOCITY...]");
        }

        telemetry.addLine("────────────────────────────────────────");

        // Distance & Aim Mode
        telemetry.addData("Mode", aimMode == AimMode.ANALYTICAL_ODOMETRY
                ? "ODOMETRY POSE (Auto-Distance)"
                : String.format(Locale.US, "FIXED DISTANCE PRESET [%.0f\"]", currentDistanceInches));
        telemetry.addData("Distance", String.format(Locale.US, "%.1f in  (%.1f cm)",
                currentDistanceInches, currentDistanceInches * 2.54));

        // Speed Metrics
        telemetry.addData("Base RPM (Kinematic)", String.format(Locale.US, "%.0f RPM", calculatedBaseRPM));
        telemetry.addData("Target RPM (Tuned)", String.format(Locale.US, "%.0f RPM", calculatedTargetRPM));
        telemetry.addData("Current Flywheel RPM", String.format(Locale.US, "%.0f RPM (Error: %+.0f RPM)", currentRPM, rpmError));

        telemetry.addLine("────────────────────────────────────────");

        // Live Tunable Constants
        telemetry.addLine("CURRENT TUNING CONSTANTS:");
        telemetry.addData("  COMPROMISE_MULTIPLIER", String.format(Locale.US, "%.3f  [D-Pad Up/Down]", COMPROMISE_MULTIPLIER));
        telemetry.addData("  COMPROMISE_OFFSET_RPM", String.format(Locale.US, "%+.1f RPM  [D-Pad Left/Right]", COMPROMISE_OFFSET_RPM));
        telemetry.addData("  Fine Mode (Left Bumper)", gamepad1.left_bumper ? "ACTIVE (Fine ±0.001 / ±2 RPM)" : "COARSE (±0.005 / ±10 RPM)");

        telemetry.addLine("────────────────────────────────────────");

        // Copy-paste snippet for team constants file
        telemetry.addLine("COPY-PASTE CONSTANTS SNIPPET:");
        telemetry.addLine(String.format(Locale.US, "public static double COMPROMISE_MULTIPLIER = %.3f;", COMPROMISE_MULTIPLIER));
        telemetry.addLine(String.format(Locale.US, "public static double COMPROMISE_OFFSET_RPM = %.1f;", COMPROMISE_OFFSET_RPM));

        telemetry.addLine("────────────────────────────────────────");
        telemetry.addData("Robot Pose", String.format(Locale.US, "(%.1f, %.1f) %.1f°",
                robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));
        telemetry.addData("Loop Rate", String.format(Locale.US, "%.1f ms (%.0f Hz)",
                loopTimer.milliseconds(), 1000.0 / Math.max(1.0, loopTimer.milliseconds())));
        telemetry.addLine("════════════════════════════════════════");
    }
}
