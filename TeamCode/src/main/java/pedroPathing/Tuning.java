package pedroPathing;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import pedroPathing.procedures.ForesightTuner;
import pedroPathing.procedures.MecanumTuner;
import pedroPathing.procedures.PinpointTuner;

/**
 * Registers the Pedro 3 AutoTune procedures (vendored from the Pedro-Pathing
 * Quickstart repo's pedro/procedures package). Once this is built and deployed to
 * the Robot Controller, AutoTune serves a browser UI at http://192.168.43.1:10158
 * on a device connected to the robot's network — there is no separate OpMode to run.
 */
public class Tuning {
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig)
        );
    }
}
