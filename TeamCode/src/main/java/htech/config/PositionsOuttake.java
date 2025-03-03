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
    public static double specimenBar = 0.45;
    public static double transferBar = 0.2;
    public static double sampleBar = 0.65;
    public static double specimenCollectBar = 0.95;
    public static double afterTransferBar = 0.4;
    public static double specimenPrescoreBar = 0.3;

    // OUTTAKE FUNNY
    public static double extendedFunny = 0.79;
    public static double retractedFunny = 0.51;
    public static double halfExtendedFunny = 0.65;
}