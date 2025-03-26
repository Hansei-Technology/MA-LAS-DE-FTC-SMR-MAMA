package htech.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class PositionsLift {
    public static int ground = 0;
    public static int park = 340;
    public static int transfer = 0;
    public static int highChamber = 520;
    public static int scoreSpecimen = 800;
    public static int highBasket = 1230;
    public static int lowBasket = 490;

    public static double kP = 0.02;
    public static double kP2 = 0.021;
    public static double kI = 0;
    public static double kD = 0;
}    
