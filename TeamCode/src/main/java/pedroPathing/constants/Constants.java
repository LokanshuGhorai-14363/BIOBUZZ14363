package pedroPathing.constants;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("leftFront");
                c.backLeftName.set("leftRear");
                c.frontRightName.set("rightFront");
                c.backRightName.set("rightRear");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.offsetUnits.set(DistanceUnit.MM);
                c.yPodOffset.set(-168.0);
                c.xPodOffset.set(-84.0);
                
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
            }
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
