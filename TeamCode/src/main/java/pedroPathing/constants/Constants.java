package pedroPathing.constants;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(config -> {
        config.frontLeftName.set("leftFront");
        config.backLeftName.set("leftRear");
        config.frontRightName.set("rightFront");
        config.backRightName.set("rightRear");

        config.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        config.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        config.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        config.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(config -> {
        config.name.set("pinpoint");
        config.xPodOffset.set(-84.0);
        config.yPodOffset.set(-168.0);
        config.offsetUnits.set(DistanceUnit.MM);
        config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        config.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(config -> {
        config.forwardTranslational.set(Controller.pid(0.1, 0.0, 0.01));
        config.strafeTranslational.set(Controller.pid(0.1, 0.0, 0.01));
        config.headingFeedback.set(Controller.pid(2.0, 0.0, 0.1));
    });

    public static Follower create(HardwareMap h) {
        return new Follower(
            new PinpointLocalizer(h, localizerConfig),
            new Mecanum(h, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }
}
