package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        PinpointConstants.forwardY = 2.7;
        PinpointConstants.strafeX = -2.95;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;


//        TwoWheelConstants.forwardTicksToInches = .001989436789;
//        TwoWheelConstants.strafeTicksToInches = .001989436789;
//        TwoWheelConstants.forwardY = 2.7;
//        TwoWheelConstants.strafeX = -2.95;
//        TwoWheelConstants.forwardEncoder_HardwareMapName = "m3";
//        TwoWheelConstants.strafeEncoder_HardwareMapName = "m0";
//        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
//        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
//        TwoWheelConstants.IMU_HardwareMapName = "imuADA";
//        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

    }
}




