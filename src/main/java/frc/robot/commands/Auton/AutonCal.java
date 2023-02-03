package frc.robot.commands.Auton;

import frc.robot.util.Vector;

public class AutonCal {
    public class AutonPos {
        public Vector xy;
        public double theta;

        public AutonPos(Vector xy, double theta){
            this.xy = xy;
            this.theta = theta;
        }

        public AutonPos(double x, double y, double theta){
            this.xy = Vector.fromXY(x, y);
        }

        public AutonPos offset(double xOffset, double yOffset, double thetaOffset){
            xy.add(Vector.fromXY(xOffset, 0));
            xy.add(Vector.fromXY(0, yOffset));
            theta += thetaOffset;

            return this;
        }
    }

    public static double maxAccel = 0;
    public static double maxVel = 0;
    public static double kP_MP = 0;
    public static double kA = 0;
    public static double kV = 0;
    public static double kS = 0;


    //-------- Positions Stuff --------//
    
    public final AutonPos[] START_POSITIONS = {
        new AutonPos(0, 0, 0),//Rightmost scoring spot
        new AutonPos(0, 0, 0),
        new AutonPos(0, 0, 0),
        new AutonPos(0, 0, 0),
        new AutonPos(0, 0, 0),
        new AutonPos(0, 0, 0),
        new AutonPos(0, 0, 0),
        new AutonPos(0, 0, 0),
        new AutonPos(0, 0, 0) //Leftmost scoring spot
    };

    public final AutonPos APRIL_RED_LEFT = new AutonPos(0, 0, 0);
    public final AutonPos APRIL_RED_COMMUNITY = new AutonPos(0, 0, 0);
    public final AutonPos APRIL_RED_RIGHT = new AutonPos(0, 0, 0);

    public final AutonPos APRIL_RED_SUBSTATION = new AutonPos(0, 0, 0);

    public final AutonPos APRIL_BLUE_LEFT = new AutonPos(0, 0, 0);
    public final AutonPos APRIL_BLUE_COMMUNITY = new AutonPos(0, 0, 0);
    public final AutonPos APRIL_BLUE_RIGHT = new AutonPos(0, 0, 0);

    public final AutonPos APRIL_BLUE_SUBSTATION = new AutonPos(0, 0, 0);
}
