package frc.robot.commands.Auton;

import frc.robot.util.Vector;

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


    public static final AutonPos[][] DRIVE_POSITIONS = {
        {//Scoring positions
            new AutonPos(0, 0, 0),//closest to wall
            new AutonPos(0, 0, 0),//
            new AutonPos(0, 0, 0),//
            new AutonPos(0, 0, 0),//
            new AutonPos(0, 0, 0),//
            new AutonPos(0, 0, 0),//
            new AutonPos(0, 0, 0),//
            new AutonPos(0, 0, 0),//
            new AutonPos(0, 0, 0),//closest to substation
        },

        {//First drive
            new AutonPos(0, 0, 0),//left of charge station
            new AutonPos(0, 0, 0),//in front of charge station
            new AutonPos(0, 0, 0),//right of charge station
        },

        {//In front of mid-field game pieces
            new AutonPos(0, 0, 0),//left piece
            new AutonPos(0, 0, 0),//left middle
            new AutonPos(0, 0, 0),//right middle
            new AutonPos(0, 0, 0),//right piece
        },

        {//Just drive out of the community zone
            new AutonPos(0, 0, 0),//left of charge station
            new AutonPos(0, 0, 0),//over charge station
            new AutonPos(0, 0, 0),//right of charge station
        }
    };

    public static final AutonPos[][] DRIVE_ONLY = {DRIVE_POSITIONS[1], DRIVE_POSITIONS[3]};

    public static final AutonPos[][] ONE_BALL_PARK = {{DRIVE_POSITIONS[1][1]}};

    public static final AutonPos[][] TWO_BALL = {DRIVE_POSITIONS[1],//first drive, first part
                                          DRIVE_POSITIONS[2],//first drive, drive to piece
                                          DRIVE_POSITIONS[1],//second drive, first part
                                          DRIVE_POSITIONS[0],//second drive, drive to score position
                                         };
}
