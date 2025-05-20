package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing Servo assigned slots specified in the Control Hub.
 */
@Config
public abstract class Servos {
    public static String outtakeLeft = "s3e";
    public static String outtakeRight = "s4e";

    public static String outtakeClaw = "s2e";

    public static String intakeBarServoLeft = "s1";
    public static String intakeBarServoRight = "s4";

    public static String intakeJointServo = "s1e";
    public static String intakeRotationServo = "s0";
    public static String intakeClawServo = "s0e";


//    public static String hangLeftServo = "s5";
//    public static String hangRightServo = "";

    public static String outtakeFunny = "s2";
}
