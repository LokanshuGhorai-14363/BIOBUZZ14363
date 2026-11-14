package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import Subsystems.VisionSubsystem.Alliance;

/**
 * DriveSubsystem — High-speed Mecanum drive matching BasicFieldCentricTeleOp performance.
 *
 * <p>Drivetrain motors are driven directly with RUN_WITHOUT_ENCODER and BRAKE mode using
 * fast kinematic math for maximum speed and zero lag. Pose tracking reads the goBILDA
 * Pinpoint directly (no Pedro Follower; Pedro is only used in auton).</p>
 */
public class DriveSubsystem extends SubsystemBase {

    // Pinpoint setup — keep in sync with Constants.localizerConfig
    private static final double X_POD_OFFSET_IN = 7.340251892570436;
    private static final double Y_POD_OFFSET_IN = 2.6424675288162836;

    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;
    private Pose currentPose = new Pose(0, 0, 0);

    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    private Alliance currentAlliance = Alliance.RED;

    /**
     * @param hardwareMap the robot's HardwareMap
     * @param telemetry   driver-station telemetry for debug output
     */
    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Motor directions matching BasicFieldCentricTeleOp
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[]{leftFront, rightFront, leftRear, rightRear}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET_IN, Y_POD_OFFSET_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU(); // robot must be still
        setPose(new Pose(0, 0, 0));
    }

    private double headingOffset = 0.0;

    /**
     * Configures the active Alliance for field-centric orientation perspective.
     *
     * @param alliance RED or BLUE
     */
    public void setAlliance(Alliance alliance) {
        this.currentAlliance = alliance;
    }

    /**
     * Direct robot-centric mecanum drive.
     *
     * @param forward  robot forward power (+1 = forward)
     * @param strafe   robot strafe power (+1 = strafe left)
     * @param rotation robot rotation power (+1 = turn CCW)
     */
    public void drive(double forward, double strafe, double rotation) {
        driveMotors(forward, strafe, rotation);
    }

    /**
     * Field-centric mecanum drive matching driver perspective for both Red and Blue Alliances.
     *
     * <p>Red Alliance (driver at X=0 looking +X):
     *   Forward (+stick Y) = +X field direction
     *   Left (+stick X)    = +Y field direction</p>
     *
     * <p>Blue Alliance (driver at X=144 looking -X):
     *   Forward (+stick Y) = -X field direction
     *   Left (+stick X)    = -Y field direction</p>
     *
     * @param forward  driver forward input (+1 = push away from driver into field)
     * @param strafe   driver strafe input (+1 = driver left)
     * @param rotation driver rotation input (+1 = turn CCW)
     */
    public void driveFieldCentric(double forward, double strafe, double rotation) {
        double fieldX;
        double fieldY;

        if (currentAlliance == Alliance.RED) {
            fieldX = forward;
            fieldY = strafe;
        } else {
            fieldX = -forward;
            fieldY = -strafe;
        }

        // Relative heading w.r.t driver starting perspective
        double heading = getHeading() - headingOffset;
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        // Transform field vector to robot local frame
        double robotX =  fieldX * cos + fieldY * sin;
        double robotY = -fieldX * sin + fieldY * cos;

        driveMotors(robotX, robotY, rotation);
    }

    /**
     * Computes direct mecanum motor powers matching BasicFieldCentricTeleOp.
     */
    private void driveMotors(double robotX, double robotY, double rotation) {
        double fl = robotX - robotY - rotation;
        double fr = robotX + robotY + rotation;
        double bl = robotX + robotY - rotation;
        double br = robotX - robotY + rotation;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftRear.setPower(bl);
        rightRear.setPower(br);
    }

    /**
     * Returns the current robot heading in radians from the Pinpoint localizer.
     */
    public double getHeading() {
        return currentPose.heading();
    }

    /**
     * Resets the driver heading reference to the robot's current orientation.
     */
    public void resetHeading() {
        pinpoint.update();
        headingOffset = currentPose.heading();
    }

    /**
     * Stops all drivetrain motion.
     */
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    @Override
    public void periodic() {
        pinpoint.update();
        currentPose = new Pose(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS));

        Pose pose = currentPose;
        if (telemetry != null) {
            telemetry.addData("Drive X", "%.2f", pose.x());
            telemetry.addData("Drive Y", "%.2f", pose.y());
            telemetry.addData("Drive Heading (°)", "%.2f", Math.toDegrees(pose.heading()));
        }
    }

    public Pose getPose() {
        return currentPose;
    }

    public void setPose(Pose pose) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pose.x(), pose.y(), AngleUnit.RADIANS, pose.heading()));
        currentPose = pose;
        headingOffset = pose.heading();
    }

    public void resumeTeleOp() {
        stop();
    }
}
