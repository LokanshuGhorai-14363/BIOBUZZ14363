package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Constants.EnumConstants;
import Constants.OdometryConstants;

@TeleOp(name = "Blue TeleOp")
public class BlueTeleOp extends TeleOpTemplate {
    @Override
    public void initialize() {
        initForAlliance(EnumConstants.AllianceColor.Blue, OdometryConstants.blueStartPoint);
    }
}
