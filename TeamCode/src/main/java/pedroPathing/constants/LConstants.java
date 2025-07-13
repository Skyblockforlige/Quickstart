package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = .0005463;
        TwoWheelConstants.strafeTicksToInches = 0.0005463;
        TwoWheelConstants.forwardY = 0;
        TwoWheelConstants.strafeX = -8.5;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "front_left";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "back_left";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    }
}




