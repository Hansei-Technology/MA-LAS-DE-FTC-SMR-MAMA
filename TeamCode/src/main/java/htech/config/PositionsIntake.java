package htech.config;

import com.acmerobotics.dashboard.config.Config;

/**
 * Class used for storing set positions for intake subsystems.
 */
@Config
public abstract class PositionsIntake {
    // INTAKE CLAW //
    public static double closedClaw = 0.319;
    public static double openedClaw = 0.65;

    // INTAKE ROTATION //
//    public static double normalRotation = 0.275;
//    public static double perpendicularRotation = 0.56;
//    public static double flippedNormalRotation = 0.84;

    public static double normalRotation = 0.083;
    public static double perpendicularRotation = 0.3;
    public static double flippedNormalRotation = 0.655;

    public static double rotation30Deg = 0.095;
    public static double rotSpeed = 0.06;
    public static double rotationAuto = 0.783;

    // INTAKE BAR //
    public static double groundPositionBar = 0.4; // over the samples
    public static double wallPositionBar = 0.52;
    public static double transferPositionBar = 0.73; // maybe change this
    public static double collectPositionBar = 0.38; // collect position = lower than ground
    public static double offsetBar = 0; //difference between the two bars

    // INTAKE JOINT //
    public static double groundPositionJoint = 0.24;
    public static double wallPickupPositionJoint = 0.44;
    public static double prepTransferPositionJoint = 0.69;
    public static double finalTransferPositionJoint = 0.74;
    public static double collectPositionJoint = 0.24;

}