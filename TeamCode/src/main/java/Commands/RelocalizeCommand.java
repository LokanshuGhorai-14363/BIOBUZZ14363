package Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import Subsystems.DriveSubsystem;
import Subsystems.VisionSubsystem;

/**
 * RelocalizeCommand — Immediately resets the robot's Pinpoint odometry pose
 * to the known Top-Left Square position (12.0in, 132.0in, 270°).
 *
 * Triggered by D-Pad UP when the robot is placed in the top-left square.
 */
public class RelocalizeCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final Telemetry telemetry;

    public RelocalizeCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, Telemetry telemetry) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.telemetry = telemetry;
    }

    public RelocalizeCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this(driveSubsystem, visionSubsystem, null);
    }

    @Override
    public void initialize() {
        Pose relocalizePose = visionSubsystem.getTopLeftRelocalizePose();
        driveSubsystem.setPose(relocalizePose);

        if (telemetry != null) {
            telemetry.addData(">> STATUS", "RELOCALIZED TO TOP-LEFT SQUARE");
            telemetry.addData(">> New Pose", String.format("(%.1fin, %.1fin, %.1f°)", 
                    relocalizePose.x(), relocalizePose.y(), Math.toDegrees(relocalizePose.heading())));
            telemetry.update();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
