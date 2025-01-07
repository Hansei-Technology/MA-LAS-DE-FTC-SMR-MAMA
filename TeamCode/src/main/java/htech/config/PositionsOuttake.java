package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for outtake subsystems.
 */
@Config
public abstract class PositionsOuttake {

    // OUTTAKE CLAW 
    public static double closedClaw = 0.48;
    public static double openedClaw = 0.34;

    // OUTTAKE BAR 
//    public static double transfer = 0.5;
//    public static double specimen = 0.5;
//    public static double score = 0.5;

    // OUTTAKE JOINT

    public static double jointSpecimenLeft = 0.25;
    public static double jointSpecimenRight = 0.83;
    public static double jointTransferLeft = 0.35;
    public static double jointTransferRight = 0.95;
    public static double jointBasketLeft = 0.428;
    public static double jointBasketRight = 0.453;
    public static double jointDropLeft = 0.2;
    public static double jointDropRight = 0.78;
    public static double jointCatapultareLeft = 0.619;
    public static double jointCatapultareRight = 0.651;

    public static double jointRotation90 = -0.155;
}