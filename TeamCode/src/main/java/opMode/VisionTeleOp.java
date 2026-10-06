package opMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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

        driverGamepad = new GamepadEx(gamepad1);

        // 3. Default Drive Command
        driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
                driveSubsystem,
                () -> -driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX()
        ));

        // 4. Default Turret Command (Auto-tracking Limelight target)
        turretSubsystem.setDefaultCommand(new RunCommand(
                () -> {
                    double targetAngle = visionSubsystem.calculateRequiredTurretAngle();
                    turretSubsystem.setAngle(targetAngle);
                },
                turretSubsystem
        ));

        // 5. Button Bindings
        // D-Pad UP for Relocalization
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> new RelocalizeCommand(driveSubsystem, visionSubsystem, alliance).schedule());

        telemetry.addData(">> Vision TeleOp Initialized for Alliance", alliance);
        telemetry.update();
    }

    @Override
    public void run() {
        // Clear bulk cache for this cycle
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        //FTC lib crashes when clock speed over 1.5 ghz
        //tim.sleep(3000) (3sec)
        // Run the FTCLib CommandScheduler
        super.run();

        telemetry.update();
    }
}
