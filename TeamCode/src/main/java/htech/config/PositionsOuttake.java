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

    public static double jointSpecimenLeft = 0.595;
    public static double jointSpecimenRight = 0.275;
    public static double jointTransferLeft = 0.37; //-0.085
    public static double jointTransferRight = 0.52; // -0.025
    public static double jointBasketLeft = 0.6;
    public static double jointBasketRight = 0.307;
    public static double jointDropLeft = 0.725;
    public static double jointDropRight = 0.312;
    public static double jointCatapultareLeft = 0.619;
    public static double jointCatapultareRight = 0.651;

    public static double jointTransferSampleLeft = 0.365;
    public static double jointTransferSampleRight = 0.525;

    public static double afterTransferLeft = 0.475;
    public static double afterTransferRight = 0.435;

    public static double jointRotation90 = -0.23;
}