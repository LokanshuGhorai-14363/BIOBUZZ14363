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
 * twoTiptwoFlowerRed — Autonomous OpMode for Red Alliance.
 *
 * <h3>Routine Sequence</h3>
 * <ol>
 *   <li><b>Start</b>: Shoots entire initial load (flywheel + transfer + intake).</li>
 *   <li><b>Path 1</b>: Drives to Flower 1 position.</li>
 *   <li><b>Flower 1 Intake</b>: Rotates Flower Servo 90° CCW, runs intake to collect, then rotates back 90° CW.</li>
 *   <li><b>Path 2</b>: Drives to position 2.</li>
 *   <li><b>Score 2</b>: Shoots full load (flywheel + transfer + intake).</li>
 *   <li><b>Path 3</b>: Drives to Flower 2 position.</li>
 *   <li><b>Flower 2 Intake</b>: Rotates Flower Servo 90° CCW, runs intake to collect, then rotates back 90° CW.</li>
 *   <li><b>Path 4</b>: Drives to final position while shooting on the move!</li>
 *   <li><b>End</b>: Saves final pose to {@link PoseStorage#currentPose} for seamless TeleOp handoff.</li>
 * </ol>
 */
@Autonomous(name = "twoTiptwoFlowerRed", group = "Autonomous")
public class twoTiptwoFlowerRed extends OpMode {

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
    private static final long SHOOT_DURATION_MS         = 1750;
    private static final long FLOWER_INTAKE_DURATION_MS = 1200;
    private static final long SERVO_RAISE_DURATION_MS   = 300;

    // ── Poses & Waypoints ─────────────────────────────────────────────────────
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose     = poseFactory.of(56.0, 8.0, 90.0);
    private final Pose path1End      = poseFactory.of(10.5, 47.7038, 180.0); // Deeper entry into Flower 1
    private final Pose path1Control1 = poseFactory.of(51.7088, 48.0897, 0.0);

    private final Pose point2End      = poseFactory.of(48.7992, 111.7742, -90.0);
    private final Pose point2Control1 = poseFactory.of(27.2544, 111.1159, 0.0);

    private final Pose point3End      = poseFactory.of(48.239, 130.5, 90.0); // Deeper entry into Flower 2
    private final Pose point3Control1 = poseFactory.of(53.1541, 119.7052, 0.0);

    private final Pose point4End      = poseFactory.of(9.7436, 117.5334, 0.0);
    private final Pose point4Control1 = poseFactory.of(61.3718, 100.8893, 0.0);

    // ── Trajectory Paths ──────────────────────────────────────────────────────
    private Path path1;
    private Path path2;
    private Path path3;
    private Path path4;

    // ── Autonomous State Machine ──────────────────────────────────────────────
    public enum PathState {
        START,
        SHOOT_INITIAL_LOAD,
        PATH_1,
        INTAKE_FLOWER_1_LOWER,
        INTAKE_FLOWER_1_RAISE,
        PATH_2,
        SHOOT_PATH2_LOAD,
        PATH_3,
        INTAKE_FLOWER_2_LOWER,
        INTAKE_FLOWER_2_RAISE,
        PATH_4_SHOOT_WHILE_MOVING,
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

        // 4. Build PedroPathing trajectory paths
        buildPaths();

        telemetry.addLine(">> twoTiptwoFlowerRed Autonomous Initialized!");
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
        telemetry.addData("Flower Servo", flowerSubsystem.is90Deg() ? "90° (CCW Deployed)" : "0° (CW Raised)");
        telemetry.addData("Shooter RPM", "%.0f / %.0f", turretShooterSubsystem.getCurrentRPM(), turretShooterSubsystem.getTargetRPM());
        telemetry.addData("Elapsed Time (s)", String.format(Locale.US, "%.2f", opModeTimer.seconds()));
        telemetry.update();
    }

    private void buildPaths() {
        path1 = Paths.curve(startPose, path1Control1, path1End)
                .heading(Interpolator.linear(startPose, path1End));

        path2 = Paths.curve(path1End, point2Control1, point2End)
                .heading(Interpolator.linear(path1End, point2End));

        path3 = Paths.curve(point2End, point3Control1, point3End)
                .heading(Interpolator.linear(point2End, point3End));

        path4 = Paths.curve(point3End, point4Control1, point4End)
                .heading(Interpolator.linear(point3End, point4End));
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
                    setPathState(PathState.PATH_1);
                }
                break;

            case PATH_1:
                // Pre-deploy flower servo & start intake while driving Path 1 (transfer stopped)
                flowerSubsystem.setAngle90();
                intakeSubsystem.spinIn(1.0);
                transferSubsystem.stop();

                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(PathState.INTAKE_FLOWER_1_LOWER);
                }
                break;

            case INTAKE_FLOWER_1_LOWER:
                // 2. End of Path 1: Continue intake at Flower 1 position (transfer stopped)
                flowerSubsystem.setAngle90();
                intakeSubsystem.spinIn(1.0);
                transferSubsystem.stop();

                if (stateTimer.milliseconds() >= FLOWER_INTAKE_DURATION_MS) {
                    setPathState(PathState.INTAKE_FLOWER_1_RAISE);
                }
                break;

            case INTAKE_FLOWER_1_RAISE:
                // Lift Flower Servo back up & stop intake
                flowerSubsystem.setAngle0();
                intakeSubsystem.stop();
                transferSubsystem.stop();

                if (stateTimer.milliseconds() >= SERVO_RAISE_DURATION_MS) {
                    setPathState(PathState.PATH_2);
                }
                break;

            case PATH_2:
                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(PathState.SHOOT_PATH2_LOAD);
                }
                break;

            case SHOOT_PATH2_LOAD:
                // 3. End of Path 2: Lower flower mech DOWN, auto-aim turret at Red Goal and shoot
                flowerSubsystem.setAngle90();
                trackGoalAndAim();
                transferSubsystem.spinProportional(0.75);
                intakeSubsystem.spinIn(1.0);

                if (stateTimer.milliseconds() >= SHOOT_DURATION_MS) {
                    intakeSubsystem.stop();
                    transferSubsystem.stop();
                    turretShooterSubsystem.stop();
                    setPathState(PathState.PATH_3);
                }
                break;

            case PATH_3:
                // Pre-deploy flower servo & start intake while driving Path 3 (transfer stopped)
                flowerSubsystem.setAngle90();
                intakeSubsystem.spinIn(1.0);
                transferSubsystem.stop();

                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(PathState.INTAKE_FLOWER_2_LOWER);
                }
                break;

            case INTAKE_FLOWER_2_LOWER:
                // 4. End of Path 3: Continue intake at Flower 2 position (transfer stopped)
                flowerSubsystem.setAngle90();
                intakeSubsystem.spinIn(1.0);
                transferSubsystem.stop();

                if (stateTimer.milliseconds() >= FLOWER_INTAKE_DURATION_MS) {
                    setPathState(PathState.INTAKE_FLOWER_2_RAISE);
                }
                break;

            case INTAKE_FLOWER_2_RAISE:
                // Lift Flower Servo back up & stop intake
                flowerSubsystem.setAngle0();
                intakeSubsystem.stop();
                transferSubsystem.stop();

                if (stateTimer.milliseconds() >= SERVO_RAISE_DURATION_MS) {
                    setPathState(PathState.PATH_4_SHOOT_WHILE_MOVING);
                }
                break;

            case PATH_4_SHOOT_WHILE_MOVING:
                // 5. Path 4: Lower flower mech DOWN, auto-aim turret at Red Goal and shoot while moving!
                flowerSubsystem.setAngle90();
                trackGoalAndAim();
                transferSubsystem.spinProportional(0.75);
                intakeSubsystem.spinIn(1.0);

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
                follower.hold(point4End);
                break;
        }
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        stateTimer.reset();

        switch (newState) {
            case PATH_1:
                flowerSubsystem.setAngle90();
                intakeSubsystem.spinIn(1.0);
                transferSubsystem.stop();
                follower.follow(path1);
                break;
            case PATH_2:
                follower.follow(path2);
                break;
            case PATH_3:
                flowerSubsystem.setAngle90();
                intakeSubsystem.spinIn(1.0);
                transferSubsystem.stop();
                follower.follow(path3);
                break;
            case PATH_4_SHOOT_WHILE_MOVING:
                follower.follow(path4);
                break;
            default:
                break;
        }
    }

    private void trackGoalAndAim() {
        Pose currentPose = follower.pose();
        visionSubsystem.getGoalTracker().update(currentPose);
        double targetTurretAngle = visionSubsystem.calculateOdometryTurretAngle(currentPose);
        turretSubsystem.setAngle(targetTurretAngle);

        double suggestedVel = visionSubsystem.getGoalTracker().getSuggestedLaunchVelocity();
        double velocity = (Double.isNaN(suggestedVel) || suggestedVel < TurretShooterSubsystem.outtakeVelocityShort)
                ? TurretShooterSubsystem.outtakeVelocityShort
                : suggestedVel;
        turretShooterSubsystem.setTargetVelocity(velocity);
    }
}
