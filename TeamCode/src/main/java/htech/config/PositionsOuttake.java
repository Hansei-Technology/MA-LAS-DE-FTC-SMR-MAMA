package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for outtake subsystems.
 */
@Config
public abstract class PositionsOuttake {

    // OUTTAKE CLAW 
    public static double closedClaw = 0.865;
    public static double openedClaw = 1;

    // OUTTAKE BAR
    public static double specimenBar = 0.5;
    public static double transferBar = 0.26;
    public static double sampleBar = 0.86;
    public static double specimenCollectBar = 0.99;
    public static double afterTransferBar = 0.4;
    public static double specimenPrescoreBar = 0.26;

    // OUTTAKE FUNNY
    public static double extendedFunny = 1;
    public static double retractedFunny = 0.87;
    public static double halfExtendedFunny = 0.82;
}