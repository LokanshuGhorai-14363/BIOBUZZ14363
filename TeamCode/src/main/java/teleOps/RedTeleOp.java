package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Constants.EnumConstants;
import Constants.OdometryConstants;

@TeleOp(name = "Red TeleOp")
public class RedTeleOp extends TeleOpTemplate {
    @Override
    public void initialize() {
        initForAlliance(EnumConstants.AllianceColor.Red, OdometryConstants.redStartPoint);
    }
}
