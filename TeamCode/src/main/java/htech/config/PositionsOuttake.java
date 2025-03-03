package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for outtake subsystems.
 */
@Config
public abstract class PositionsOuttake {

    // OUTTAKE CLAW 
    public static double closedClaw = 0.34;
    public static double openedClaw = 0.15;

    // OUTTAKE BAR
    public static double specimenBar = 0.5;
    public static double transferBar = 0.8;
    public static double sampleBar = 0.3;
    public static double specimenCollectBar = 0.6;
    public static double afterTransferBar = 0.7;

    // OUTTAKE FUNNY
    public static double extendedFunny = 0.5;
    public static double retractedFunny = 0.8;
    public static double halfExtendedFunny = 0.65;
}