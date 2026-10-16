package teleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.EnumConstants;
import Constants.OdometryConstants;
import Constants.RobotConstants;
import subsystems.MecanumDrive;
import utility.RobotHardware;

/**
 * Shared TeleOp lifecycle. Alliance subclasses only call initForAlliance(...) from
 * initialize(). Restores pose from the end of auto (OdometryConstants.endingAutonPose)
 * when present, otherwise falls back to the given alliance start pose.
 */
public abstract class TeleOpTemplate extends CommandOpMode {
    protected RobotHardware robot;
    protected Follower follower;
    protected MecanumDrive mecanumDrive;

    private final ElapsedTime telemetryTimer = new ElapsedTime();

    protected void initForAlliance(EnumConstants.AllianceColor allianceColor, Pose fallbackStartPose) {
        // The scheduler is a static singleton and SubsystemBase self-registers in its
        // constructor, so a previous OpMode that died before CommandOpMode.reset() would
        // leave subsystems holding a dead Follower registered here. Clear it first.
        CommandScheduler.getInstance().reset();

        RobotConstants.Robot.allianceColor = allianceColor;

        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);
        follower = robot.follower;

        Pose startPose = OdometryConstants.endingAutonPose != null
                ? OdometryConstants.endingAutonPose
                : fallbackStartPose;
        follower.setPose(startPose);

        mecanumDrive = new MecanumDrive();

        configureButtonBindings();

        telemetryTimer.reset();
    }

    protected void configureButtonBindings() {
        // Pedro's mecanum mixer treats +strafe as LEFT and +turn as counter-clockwise,
        // while both gamepad X axes are positive to the RIGHT, so both are negated.
        mecanumDrive.setDefaultCommand(new RunCommand(() -> mecanumDrive.drive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ), mecanumDrive));
    }

    @Override
    public void run() {
        robot.clearBulkCache();
        follower.update();
        robot.updateCachedPose();

        super.run();

        if (telemetryTimer.milliseconds() >= 200) {
            telemetry.addData("Alliance", RobotConstants.Robot.allianceColor);
            telemetry.addData("Pose", "(%.2f, %.2f, %.1f deg)",
                    robot.cachedPoseX, robot.cachedPoseY, Math.toDegrees(robot.cachedPoseHeading));
            telemetry.update();
            telemetryTimer.reset();
        }
    }
}
