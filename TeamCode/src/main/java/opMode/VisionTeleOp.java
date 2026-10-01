package opMode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

import Commands.DefaultDriveCommand;
import Commands.RelocalizeCommand;
import Subsystems.DriveSubsystem;
import Subsystems.TurretSubsystem;
import Subsystems.VisionSubsystem;
import Subsystems.VisionSubsystem.Alliance;

/**
 * VisionTeleOp — Advanced TeleOp featuring Vision targeting and PedroPathing relocalization.
 */
@TeleOp(name = "Vision TeleOp", group = "Competition")
public class VisionTeleOp extends CommandOpMode {

    private DriveSubsystem driveSubsystem;
    private TurretSubsystem turretSubsystem;
    private VisionSubsystem visionSubsystem;

    private GamepadEx driverGamepad;
    private List<LynxModule> allHubs;
    
    private Alliance selectedAlliance = Alliance.RED; // Default
    private boolean initLoopDone = false;

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
        // Gamepad X for Blue Alliance, B for Red Alliance (Handled in init_loop mostly, but can set here)
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(() -> {
                    selectedAlliance = Alliance.BLUE;
                    visionSubsystem.setAlliance(selectedAlliance);
                });
                
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(() -> {
                    selectedAlliance = Alliance.RED;
                    visionSubsystem.setAlliance(selectedAlliance);
                });

        // D-Pad UP for Relocalization
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> new RelocalizeCommand(driveSubsystem, visionSubsystem, selectedAlliance).schedule());

        telemetry.addLine(">> Vision TeleOp Initialized.");
        telemetry.addLine(">> Press X for Blue Alliance, B for Red Alliance.");
        telemetry.update();
    }
    
    @Override
    public void run() {
        // Clear bulk cache for this cycle
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Handle init-loop Alliance selection before Start is pressed
        if (!isStarted() && !isStopRequested()) {
            driverGamepad.readButtons();
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.X)) {
                selectedAlliance = Alliance.BLUE;
                visionSubsystem.setAlliance(selectedAlliance);
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                selectedAlliance = Alliance.RED;
                visionSubsystem.setAlliance(selectedAlliance);
            }
            telemetry.addData("Selected Alliance", selectedAlliance);
            telemetry.update();
            return; // Skip running scheduler until started
        }

        // Run the FTCLib CommandScheduler
        super.run();
        
        telemetry.update();
    }
}
