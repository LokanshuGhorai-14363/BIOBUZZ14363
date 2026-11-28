package opMode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import java.util.List;
import java.util.Locale;

import Subsystems.FlowerSubsystem;
import Subsystems.IntakeSubsystem;
import Subsystems.TransferSubsystem;
import Subsystems.TurretShooterSubsystem;
import Subsystems.TurretSubsystem;
import Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import util.PoseStorage;

/**
 * ShootAndParkRed — Simple Autonomous OpMode for Red Alliance.
 *
 * <h3>Routine Sequence</h3>
 * <ol>
 *   <li><b>Start</b>: Lowers flower mechanism, auto-aims turret, and shoots initial load.</li>
 *   <li><b>Path 1 (Park)</b>: Drives to park position (10.1736", 112.1438", 0°).</li>
 *   <li><b>End</b>: Saves final pose to {@link PoseStorage#currentPose} for seamless TeleOp handoff.</li>
 * </ol>
 */
@Autonomous(name = "ShootAndParkRed", group = "Autonomous")
public class ShootAndParkRed extends OpMode {

    // ── PedroPathing & Hardware ──────────────────────────────────────────────
    private Follower follower;
    private List<LynxModule> allHubs;

    // ── Subsystems ────────────────────────────────────────────────────────────
    private FlowerSubsystem flowerSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private TurretShooterSubsystem turretShooterSubsystem;
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;

    // ── Timing Constants (ms) ────────────────────────────────────────────────
    private static final long INITIAL_SHOOT_DURATION_MS = 1750;

    // ── Poses & Waypoints ─────────────────────────────────────────────────────
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose     = poseFactory.of(56.0, 8.0, 90.0);
    private final Pose path1End      = poseFactory.of(10.1736, 112.1438, 0.0);
    private final Pose path1Control1 = poseFactory.of(15.5237, 76.9035, 0.0);

    // ── Trajectory Path ───────────────────────────────────────────────────────
    private Path path1;

    // ── Autonomous State Machine ──────────────────────────────────────────────
    public enum PathState {
        START,
        SHOOT_INITIAL_LOAD,
        PATH_1_PARK,
        PARKED
    }

    private PathState pathState = PathState.START;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime opModeTimer = new ElapsedTime();

    @Override
    public void init() {
        // 1. Configure LynxModule bulk caching for fast execution
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. Initialize PedroPathing follower & starting pose
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        // 3. Initialize robot mechanisms and subsystems
        flowerSubsystem = new FlowerSubsystem(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        transferSubsystem = new TransferSubsystem(hardwareMap, telemetry);
        turretSubsystem = new TurretSubsystem(hardwareMap, telemetry);
        turretShooterSubsystem = new TurretShooterSubsystem(hardwareMap, telemetry);
        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry, turretSubsystem, null);

        // Configure Vision Alliance perspective to Red
        visionSubsystem.setAlliance(VisionSubsystem.Alliance.RED);

        // 4. Build PedroPathing trajectory path
        buildPaths();

        telemetry.addLine(">> ShootAndParkRed Autonomous Initialized!");
        telemetry.addData("Start Pose", String.format(Locale.US, "(%.1f\", %.1f\", %.1f°)",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading())));
        telemetry.update();
    }

    @Override
    public void init_loop() {
        flowerSubsystem.setAngle0();
    }

    @Override
    public void start() {
        opModeTimer.reset();
        setPathState(PathState.SHOOT_INITIAL_LOAD);
    }

    @Override
    public void loop() {
        // 1. Clear LynxModule bulk cache at loop top
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // 2. Update PedroPathing follower and autonomous state machine
        follower.update();
        autonomousPathUpdate();

        // 3. Update mechanism periodic loops
        flowerSubsystem.periodic();
        intakeSubsystem.periodic();
        transferSubsystem.periodic();
        turretShooterSubsystem.periodic();
        turretSubsystem.periodic();
        visionSubsystem.periodic();

        // 4. Save current pose continuously so TeleOp has the exact pose if stopped early or finished
        Pose currentPose = follower.pose();
        PoseStorage.currentPose = currentPose;

        // 5. Telemetry diagnostics
        telemetry.addData("Path State", pathState);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Pose", String.format(Locale.US, "(%.1f\", %.1f\", %.1f°)",
                currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading())));
        telemetry.addData("Shooter RPM", "%.0f / %.0f", turretShooterSubsystem.getCurrentRPM(), turretShooterSubsystem.getTargetRPM());
        telemetry.addData("Elapsed Time (s)", String.format(Locale.US, "%.2f", opModeTimer.seconds()));
        telemetry.update();
    }

    private void buildPaths() {
        path1 = Paths.curve(startPose, path1Control1, path1End)
                .heading(Interpolator.linear(startPose, path1End));
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                setPathState(PathState.SHOOT_INITIAL_LOAD);
                break;

            case SHOOT_INITIAL_LOAD:
                // 1. Initial Start: Lower flower mech DOWN, auto-aim turret at Red Goal and shoot
                flowerSubsystem.setAngle90();
                trackGoalAndAim();
                transferSubsystem.spinProportional(0.75);
                intakeSubsystem.spinIn(1.0);

                if (stateTimer.milliseconds() >= INITIAL_SHOOT_DURATION_MS) {
                    intakeSubsystem.stop();
                    transferSubsystem.stop();
                    turretShooterSubsystem.stop();
                    flowerSubsystem.setAngle0();
                    setPathState(PathState.PATH_1_PARK);
                }
                break;

            case PATH_1_PARK:
                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(PathState.PARKED);
                }
                break;

            case PARKED:
                // Autonomous completed: stop mechanisms & save pose
                intakeSubsystem.stop();
                transferSubsystem.stop();
                turretShooterSubsystem.stop();
                flowerSubsystem.setAngle0();

                PoseStorage.currentPose = follower.pose();
                follower.hold(path1End);
                break;
        }
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        stateTimer.reset();

        switch (newState) {
            case PATH_1_PARK:
                follower.follow(path1);
                break;
            default:
                break;
        }
    }

    private void trackGoalAndAim() {
        Pose currentPose = follower.pose();
        util.HiveGoalTracker tracker = visionSubsystem.getGoalTracker();
        tracker.update(currentPose);

        // ── 1. Turret aiming ─────────────────────────────────────────────────
        double targetTurretAngle = visionSubsystem.calculateOdometryTurretAngle(currentPose);
        turretSubsystem.setAngle(targetTurretAngle);

        // ── 2. Ballistics-based shooter velocity ─────────────────────────────
        double distanceIn     = tracker.getDistanceInches();
        double suggestedVel   = tracker.getSuggestedLaunchVelocity();

        // Safe fallback: if the model returns NaN or an unusably low speed,
        // revert to the hardcoded outtakeVelocityShort default.
        boolean usedFallback = false;
        double velocity;
        if (Double.isNaN(suggestedVel) || suggestedVel < TurretShooterSubsystem.outtakeVelocityShort) {
            velocity = TurretShooterSubsystem.outtakeVelocityShort;
            usedFallback = true;
        } else {
            velocity = suggestedVel;
        }
        turretShooterSubsystem.setTargetVelocity(velocity);

        // ── 3. Telemetry diagnostics ─────────────────────────────────────────
        if (telemetry != null) {
            double targetRPM  = turretShooterSubsystem.getTargetRPM();
            double currentRPM = turretShooterSubsystem.getCurrentRPM();
            telemetry.addData("Goal Dist (in)", String.format(java.util.Locale.US, "%.1f", distanceIn));
            telemetry.addData("Ballistics Vel (tps)", String.format(java.util.Locale.US, "%.0f%s",
                    velocity, usedFallback ? " [FALLBACK]" : ""));
            telemetry.addData("Target / Current RPM",
                    String.format(java.util.Locale.US, "%.0f / %.0f", targetRPM, currentRPM));
        }
    }
}
