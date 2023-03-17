package frc.robot.commands.Auton;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import frc.robot.util.Vector;

public class AutonPos {
    public Vector xy;
    public double value;

    public AutonPos(Vector xy, double deg){
        this.xy = xy;
        this.value = Math.toRadians(deg);
    }

    public AutonPos(double x, double y, double value){
        this.xy = Vector.fromXY(x, y);
        this.value = Math.toRadians(value);
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

    //construct at start
    public static AprilTagFieldLayout tagLayout;
    static {
        try{
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch(Exception e){
            e.printStackTrace();
        }
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


    //Start Positions
    public static AutonPos substation = new AutonPos(54.25 + 4, 20.19 + 11.0 + 154.0 + 2, 165);
    public static AutonPos subMid = new AutonPos(54.25 + 4, 20.19 + 11.0 + 110.0 + 2, 165);
    public static AutonPos farMid = new AutonPos(54.25 + 4, 20.19 + 11.0 + 44.0 + 2, 165);
    public static AutonPos far = new AutonPos(54.25 + 4, 20.19 + 11.0 + 2, -165);

    //First Drives
    public static AutonPos driveSub = new AutonPos(86, 156.61 + 25, 0).offset(12, 0, 0);
    public static AutonPos driveFar = new AutonPos(86, 59.36 - 25, 0).offset(12, 0, 0);

    //Drive Outs
    public static AutonPos driveOutSub = new AutonPos(175, 156.61 + 25, 0);
    public static AutonPos driveOutFar = new AutonPos(230, 59.36 - 25, 0);

    //Drive To Piece
    public static AutonPos drivePieceSub = new AutonPos(266, 180.2, 0).offset(0, 3, 0);
    public static AutonPos drivePieceFar = new AutonPos(266, 36.2, 0).offset(0, 3, 0);

    //Drive To Score
    public static AutonPos driveScoreSub = new AutonPos(70.24, 174.19, 180).offset(12, 12, 0);
    public static AutonPos driveScoreFar = new AutonPos(70.24, 42.19, 180).offset(12, 12, 0);

    //Drive To Balance
    public static AutonPos driveToBalComm = new AutonPos(86, 108, 90);
    public static AutonPos driveToBalOutside = new AutonPos(230, 108, 90);

    //Arm Offset
    public static double initArmStendo = 32.5;
    public static double initArmAngle = -3.9;


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
