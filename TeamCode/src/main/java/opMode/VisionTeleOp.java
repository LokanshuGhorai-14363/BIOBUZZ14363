package opMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

import Commands.DefaultDriveCommand;
import Commands.RelocalizeCommand;
import Subsystems.DriveSubsystem;
import Subsystems.TurretSubsystem;
import Subsystems.VisionSubsystem;
import Subsystems.VisionSubsystem.Alliance;

/**
 * VisionTeleOp — Abstract base TeleOp class for Vision targeting and PedroPathing relocalization.
 */
public abstract class VisionTeleOp extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;

    private GamepadEx driverGamepad;
    private List<LynxModule> allHubs;

    private final Alliance alliance;

    public VisionTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        // 1. Enable LynxModule bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // 2. Instantiate Subsystems
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
        turretSubsystem = new TurretSubsystem(hardwareMap, telemetry);
        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry, turretSubsystem, driveSubsystem);

        register(driveSubsystem, turretSubsystem, visionSubsystem);

        // Configure alliance in VisionSubsystem
        visionSubsystem.setAlliance(alliance);

        // 3. Set robot initial start pose based on alliance:
        // Red: Back against y=0 wall, back-right corner at (X=72, Y=0) -> (63.25, 7.25, 90°)
        // Blue: Back against y=144 wall, back-right corner at (X=72, Y=144) -> (80.75, 136.75, 270°)
        Pose startPose = visionSubsystem.getStartPose(alliance);
        driveSubsystem.setPose(startPose);

        driverGamepad = new GamepadEx(gamepad1);

        // 4. Default Drive Command
        driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
                driveSubsystem,
                () -> -driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX()
        ));

        // 5. Default Turret Command (Auto-tracking Goal using Pinpoint Odometry)
        turretSubsystem.setDefaultCommand(new RunCommand(
                () -> {
                    double targetAngle = visionSubsystem.calculateOdometryTurretAngle();
                    turretSubsystem.setAngle(targetAngle);
                },
                turretSubsystem
        ));

        // 6. Button Bindings
        // D-Pad UP: Manual relocalization when placed in the top-left square
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new RelocalizeCommand(driveSubsystem, visionSubsystem, telemetry));

        telemetry.addData(">> Vision TeleOp Initialized for Alliance", alliance);
        telemetry.addData(">> Initial Pose", String.format("(%.1fin, %.1fin, %.1f°)", 
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading())));
        telemetry.update();
    }

    @Override
    public void run() {
        // Clear bulk cache for this cycle
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Run the FTCLib CommandScheduler
        super.run();

        telemetry.update();
    }
}
