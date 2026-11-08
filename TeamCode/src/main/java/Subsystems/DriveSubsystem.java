package Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathSegment;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.config.Modifier;
import pedroPathing.constants.Constants;
import Subsystems.VisionSubsystem.Alliance;

import java.util.ArrayList;
import java.util.List;

/**
 * DriveSubsystem — High-speed Mecanum drive matching BasicFieldCentricTeleOp performance.
 *
 * <p>Drivetrain motors are driven directly with RUN_WITHOUT_ENCODER and BRAKE mode using
 * fast kinematic math for maximum speed and zero lag, while Pedro Pathing Follower maintains
 * continuous Pinpoint odometry pose tracking.</p>
 */
public class DriveSubsystem extends SubsystemBase {

    private final Follower follower;
    private final Telemetry telemetry;

    private final DcMotorEx leftFront;
    private final DcMotorEx rightFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightRear;

    private Alliance currentAlliance = Alliance.RED;
    private double headingOffset = 0.0;

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

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));
    }

    /**
     * Configures the active Alliance for field-centric orientation perspective.
     *
     * @param alliance RED (driver at X=0 facing +X) or BLUE (driver at X=144 facing -X)
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
     * Field-centric mecanum drive matching BasicFieldCentricTeleOp speed.
     *
     * <p>Red Alliance (driver at X=0 facing +X): Forward = +X (right on diagram), Left = +Y (up on diagram).<br>
     * Blue Alliance (driver at X=144 facing -X): Forward = -X (left on diagram), Left = -Y (down on diagram).</p>
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

        double heading = getHeading() - headingOffset;
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double robotX = fieldX * cos - fieldY * sin;
        double robotY = fieldX * sin + fieldY * cos;

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
        return follower.pose().heading();
    }

    /**
     * Resets field-centric heading reference to current alliance orientation.
     */
    public void resetHeading() {
        headingOffset = getHeading() - (currentAlliance == Alliance.BLUE ? Math.PI : 0.0);
    }

    /**
     * Stops all drivetrain motion.
     */
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        follower.drivetrain.stop();
    }

    @Override
    public void periodic() {
        follower.update();

        Pose pose = follower.pose();
        if (telemetry != null) {
            telemetry.addData("Drive X", "%.2f", pose.x());
            telemetry.addData("Drive Y", "%.2f", pose.y());
            telemetry.addData("Drive Heading (°)", "%.2f", Math.toDegrees(pose.heading()));
        }
    }

    public Follower getFollower() {
        return follower;
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void followPathTo(Pose target) {
        Path path = new Path(new Line(follower.pose(), target), new ArrayList<>()) {
            @Override
            public double heading(double v) {
                return target.heading();
            }

            @Override
            protected boolean hasHeading() {
                return true;
            }

            @Override
            protected List<PathSegment> getSegments(PathSegment.HeadingProvider headingProvider, List<Modifier> list) {
                return new ArrayList<>();
            }

            @Override
            protected Path withHeading(Interpolator interpolator) {
                return this;
            }

            @Override
            protected Path withModifiers(List<Modifier> list) {
                return this;
            }
        }.heading(Interpolator.linear(follower.pose(), target));
        follower.follow(path);
    }

    public boolean isPathComplete() {
        return !follower.isBusy();
    }

    public void resumeTeleOp() {
        stop();
    }
}
