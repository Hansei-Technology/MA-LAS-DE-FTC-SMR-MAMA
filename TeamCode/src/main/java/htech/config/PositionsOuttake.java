package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for outtake subsystems.
 */
@Config
public abstract class PositionsOuttake {

    // OUTTAKE CLAW 
    public static double closedClaw = 0.49;
    public static double openedClaw = 0.35;

    // OUTTAKE BAR 
//    public static double transfer = 0.5;
//    public static double specimen = 0.5;
//    public static double score = 0.5;

    // OUTTAKE JOINT

    public static double jointSpecimenLeft = 0.425;
    public static double jointSpecimenRight = 0.425;
    public static double jointTransferLeft = 0.2; //+0.083
    public static double jointTransferRight = 0.2; // +0.073
    public static double jointBasketLeft = 0.4;
    public static double jointBasketRight = 0.4;
    public static double jointDropLeft = 0.725;
    public static double jointDropRight = 0.385;
    public static double jointCatapultareLeft = 0.619;
    public static double jointCatapultareRight = 0.651;

    public static double jointTransferSampleLeft = 0.48;
    public static double jointTransferSampleRight = 0.58;

    public static double afterTransferLeft = 0.25;
    public static double afterTransferRight = 0.25;

    public static double jointRotation90 = 0.23;


    public static double jointSpecimenVerticalLeft = 0.394;
    public static double jointSpecimenVerticalRight = 0.394;


    public static double specialTransferLeft = 0.08;
    public static double specialTransferRight = 0.38;
}