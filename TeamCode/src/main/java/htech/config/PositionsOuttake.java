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
    public static double specimenBar = 0.8;
    public static double transferBar = 0.64;
    public static double sampleBar = 0.25;
    public static double specimenCollectBar = 0.08;
    public static double afterTransferBar = 0.5;
    public static double specimenPrescoreBar = 0.78;

    // OUTTAKE FUNNY
    public static double extendedFunny = 0.6;
    public static double retractedFunny = 0.33;
    public static double halfExtendedFunny = 0.47;
}