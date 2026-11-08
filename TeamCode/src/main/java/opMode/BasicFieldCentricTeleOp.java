package opMode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Standalone field-centric mecanum TeleOp using only the goBILDA Pinpoint (no Pedro, no subsystems).
 *
 * <p>Robot starts at field center (72, 72) in, heading 0°. Field +X is "away from the driver"
 * at heading 0, +Y is to the left, heading is CCW-positive.</p>
 *
 * <p>Controls (gamepad1): left stick = translate, right stick X = rotate,
 * OPTIONS = reset pose to (72, 72, 0°).</p>
 */
@TeleOp(name = "Basic Field Centric", group = "Test")
public class BasicFieldCentricTeleOp extends LinearOpMode {

    private static final double START_X_IN = 72.0;
    private static final double START_Y_IN = 72.0;
    private static final double START_HEADING_DEG = 0.0;

    // Pinpoint setup — mirrors Constants.localizerConfig
    private static final double X_POD_OFFSET_IN = 7.184244591420091;
    private static final double Y_POD_OFFSET_IN = -0.3759881267397423;

    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftRear, rightRear}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(X_POD_OFFSET_IN, Y_POD_OFFSET_IN, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU(); // robot must be still

        telemetry.addLine("Keep robot still. Ready — press START.");
        telemetry.update();
        waitForStart();

        Pose2D startPose = new Pose2D(DistanceUnit.INCH, START_X_IN, START_Y_IN, AngleUnit.DEGREES, START_HEADING_DEG);
        pinpoint.setPosition(startPose);

        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            if (gamepad1.options) {
                pinpoint.setPosition(startPose);
            }

            // Driver inputs in field frame: +x forward, +y left, +turn CCW
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;

            // Rotate field-frame input into robot frame
            double heading = pose.getHeading(AngleUnit.RADIANS);
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double robotX = x * cos - y * sin;
            double robotY = x * sin + y * cos;

            double fl = robotX - robotY - turn;
            double fr = robotX + robotY + turn;
            double bl = robotX + robotY - turn;
            double br = robotX - robotY + turn;

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

            telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", "%.2f", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("leftFront", "%.3f", fl);
            telemetry.addData("rightFront", "%.3f", fr);
            telemetry.addData("leftRear", "%.3f", bl);
            telemetry.addData("rightRear", "%.3f", br);
            telemetry.addData("Loop (ms)", "%.1f  (%.0f Hz)", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0);
            telemetry.update();
        }
    }
}
