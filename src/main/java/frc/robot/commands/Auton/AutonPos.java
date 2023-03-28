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
        this.xy = new Vector(pos.xy);
        this.value = pos.value;
    }

    public AutonPos offset(double xOffset, double yOffset, double thetaOffset){
        xy.add(Vector.fromXY(xOffset, 0));
        xy.add(Vector.fromXY(0, yOffset));
        value += Math.toRadians(thetaOffset);

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
    public static AutonPos drivePieceSub = new AutonPos(266, 180.2, 0).offset(-14, 4.5, 0);
    public static AutonPos drivePieceFar = new AutonPos(266, 36.2, 0).offset(-14, 3-3, 5);

    //Drive To Score
    //public static AutonPos driveScoreSub = new AutonPos(70.24, 174.19, 180).offset(12, 12, 0);
    public static AutonPos driveScoreSub = new AutonPos(70.24, 174.19, -167).offset(-3, 6, 0);
    public static AutonPos driveScoreFar = new AutonPos(70.24, 42.19, 167).offset(-3, -6, 0);
    //public static AutonPos driveScoreFar = new AutonPos(200.0, 42.19, 180).offset(12, 12-12, 0);

    //Drive To Balance
    public static AutonPos driveToBalComm = new AutonPos(86, 108, 90);
    public static AutonPos driveToBalOutside = new AutonPos(230, 108, 90);

    //Bump Positions
    public static AutonPos preBumpPos = new AutonPos(driveFar.xy.getX(), 59.36 - 25, 0);
    public static AutonPos postBumpPos = new AutonPos(driveFar.xy.getX() + 60.0, 59.36 - 25, 180);

    //Arm Offset
    public static double initArmStendo = 32.5;
    public static double initArmAngle = -3.9;

    public static double tagToMidX = 20;

    static double postXPlusOffset = 54.25 + 20;
    static double postXPlusOffsetHiCone = 54.25 + 6;
    static double postXPlusOffsetMidLowCube = postXPlusOffset + 9;

    static double initNodeY = 20.19;
    static double nodeDist = 22.0;
    public static final AutonPos[][] SCORING_OFFSETS = {

        //Low
        {new AutonPos(postXPlusOffset, initNodeY, 180),//Wall
         new AutonPos(postXPlusOffsetMidLowCube, initNodeY + nodeDist, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 2.0, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 3.0, 180),
         new AutonPos(postXPlusOffsetMidLowCube, initNodeY + nodeDist * 4.0, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 5.0, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 6.0, 180),
         new AutonPos(postXPlusOffsetMidLowCube, initNodeY + nodeDist * 7.0, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 8.0, 180)},//Sub

        //Medium
        {new AutonPos(postXPlusOffset, initNodeY, 180),//Wall
         new AutonPos(postXPlusOffsetMidLowCube, initNodeY + nodeDist, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 2.0, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 3.0, 180),
         new AutonPos(postXPlusOffsetMidLowCube, initNodeY + nodeDist * 4.0, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 5.0, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 6.0, 180),
         new AutonPos(postXPlusOffsetMidLowCube, initNodeY + nodeDist * 7.0, 180),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 8.0, 180)},//Sub

        //High
        {new AutonPos(postXPlusOffsetHiCone, initNodeY + nodeDist / 2.0, 165),//Wall
         new AutonPos(postXPlusOffset, initNodeY + nodeDist, 180),
         new AutonPos(postXPlusOffsetHiCone, initNodeY + nodeDist * 2.0 + nodeDist / 2.0, 165),
         new AutonPos(postXPlusOffsetHiCone, initNodeY + nodeDist * 2.0 + nodeDist / 2.0 - 6.0, -165),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 4.0, 180),
         new AutonPos(postXPlusOffsetHiCone, initNodeY + nodeDist * 5.0 + nodeDist / 2.0, 165),
         new AutonPos(postXPlusOffsetHiCone, initNodeY + nodeDist * 5.0 + nodeDist / 2.0 - 6.0, -165),
         new AutonPos(postXPlusOffset, initNodeY + nodeDist * 7.0, 180),
         new AutonPos(postXPlusOffsetHiCone, initNodeY + nodeDist * 7.0 + nodeDist / 2.0 - 6.0, -165)},//Sub
    };

    public static final AutonPos GATHER_OFFSET = new AutonPos(636.960-30, 265.740, 0);
    public static final double GATHER_X_DIFF = 24;
}
