package frc.robot.commands.Auton;

import frc.robot.util.Vector;

public class AutonPos {
    public Vector xy;
    public double value;

    public AutonPos(Vector xy, double theta){
        this.xy = xy;
        this.value = theta;
    }

    public AutonPos(double x, double y, double value){
        this.xy = Vector.fromXY(x, y);
    }

    public AutonPos (AutonPos pos){
        this.xy = pos.xy;
        this.value = pos.value;
    }

    public AutonPos offset(double xOffset, double yOffset, double thetaOffset){
        xy.add(Vector.fromXY(xOffset, 0));
        xy.add(Vector.fromXY(0, yOffset));
        value += thetaOffset;

        return this;
    }

    public AutonPos mirrorY(boolean mirror){
        xy = Vector.subVectors(xy, Vector.fromXY(0, 2 * xy.getY()));

        return this;
    }

    
    public static final AutonPos[] START_POSITIONS = {
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

    public static final AutonPos APRIL_RED_LEFT = new AutonPos(0, 0, 0);
    public static final AutonPos APRIL_RED_COMMUNITY = new AutonPos(0, 0, 0);
    public static final AutonPos APRIL_RED_RIGHT = new AutonPos(0, 0, 0);

    public static final AutonPos APRIL_RED_SUBSTATION = new AutonPos(0, 0, 0);

    public static final AutonPos APRIL_BLUE_LEFT = new AutonPos(0, 0, 0);
    public static final AutonPos APRIL_BLUE_COMMUNITY = new AutonPos(0, 0, 0);
    public static final AutonPos APRIL_BLUE_RIGHT = new AutonPos(0, 0, 0);

    public static final AutonPos APRIL_BLUE_SUBSTATION = new AutonPos(0, 0, 0);


    public static final AutonPos[] SCORING_POSITIONS = {
        new AutonPos(0, 0, 0),//closest to wall
        new AutonPos(0, 0, 0),//
        new AutonPos(0, 0, 0),//
        new AutonPos(0, 0, 0),//
        new AutonPos(0, 0, 0),//
        new AutonPos(0, 0, 0),//
        new AutonPos(0, 0, 0),//
        new AutonPos(0, 0, 0),//
        new AutonPos(0, 0, 0),//closest to substation
    };
        

    public static final AutonPos[] FIRST_DRIVE = {
        new AutonPos(0, 0, 0),//left of charge station
        new AutonPos(0, 0, 0),//in front of charge station
        new AutonPos(0, 0, 0),//right of charge station
    };

    public static final AutonPos[] MID_FIELD_POS = {
        new AutonPos(0, 0, 0),//substation
        new AutonPos(0, 0, 0),//sub middle
        new AutonPos(0, 0, 0),//far middle
        new AutonPos(0, 0, 0),//far
    };

    public static final AutonPos[] JUST_DRIVE_OUT = {
        new AutonPos(0, 0, 0),//left of charge station
        new AutonPos(0, 0, 0),//over charge station
        new AutonPos(0, 0, 0),//right of charge station
    };
}
