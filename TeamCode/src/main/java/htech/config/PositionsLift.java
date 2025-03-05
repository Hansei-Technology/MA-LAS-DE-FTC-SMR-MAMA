package htech.config;

import com.acmerobotics.dashboard.config.Config;

@Config
public abstract class PositionsLift {
    public static int ground = 0;
    public static int park = 340;
    public static int transfer = 0;
    public static int highChamber = 550;
    public static int scoreSpecimen = 720;
    public static int highBasket = 1250;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
}    
