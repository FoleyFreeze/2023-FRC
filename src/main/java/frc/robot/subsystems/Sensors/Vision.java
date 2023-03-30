package frc.robot.subsystems.Sensors;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.util.LimitedStack;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEvent;
import java.nio.ByteBuffer;
import java.util.EnumSet;
import java.util.Vector;

public class Vision extends SubsystemBase {

    RobotContainer r;

    boolean debug = false;

    //TODO: remove the added offset when the camera is recal'd
    frc.robot.util.Vector camLocation = frc.robot.util.Vector.fromXY(9.75, -8.75);

    private BooleanEntry active;
    private DoubleEntry rioTime;
    private RawSubscriber poseMsg;
    private int listener;
    private ByteBuffer poseData;
    public LimitedStack<VisionDataEntry> visionProduct;
    

    public Vision(RobotContainer r) {
        super();
        this.r = r;
    }

    public LimitedStack<VisionDataEntry> init() {
        rioTime = NetworkTableInstance.getDefault().getDoubleTopic("/Vision/RIO Time").getEntry(Timer.getFPGATimestamp());
        active = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Active").getEntry(true);
        poseMsg = NetworkTableInstance.getDefault().getTable("Vision").getRawTopic("Pose Data Bytes").subscribe("raw", null);
        visionProduct = new LimitedStack<VisionDataEntry>(5);
        listener = NetworkTableInstance.getDefault().addListener(poseMsg, 
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                poseData = ByteBuffer.wrap(event.valueData.value.getRaw());
                byte type = poseData.get(12); // type 1 = tag, type 2 = cone, type 3 = cube
                byte numTags = poseData.get(13);
                if(type == 1 && numTags <= 4){
                    VisionDataEntry e = new VisionDataEntry();
                    e.listFin = new Vector<VisionData>();
                    e.seqNum = poseData.getInt(0);
                    double current = Timer.getFPGATimestamp();
                    e.timestamp = current - ((current -  poseData.getFloat(4) + poseData.getFloat(8)) / 2.0);
                    for(int i = 0, b = 14; i < numTags; i++, b += 25){
                        
                        VisionData visionData = new VisionData(type, poseData.get(b), 
                            poseData.getFloat(b+1), poseData.getFloat(b+5), poseData.getFloat(b+9), 
                            poseData.getFloat(b+13), poseData.getFloat(b+17), poseData.getFloat(b+21));
                        e.listFin.add(visionData);
                    }
                    visionProduct.push(e);
                }

            });
        return visionProduct;
    }
    @Override
    public void periodic() {

        rioTime.set(Timer.getFPGATimestamp());
        NetworkTableInstance.getDefault().flush();
    
    }

    public void activate(boolean state) {

        // put the state (true or false) into the network table topic that will acivate or deactivate the Pi
        active.set(state);

    }


    public void togglePi(){
        boolean status = active.get();
        active.set(!status);
    }

    public frc.robot.util.Vector getImageVector(int level, int position, boolean scoring){
        //if image doesnt exists
        if(visionProduct.isEmpty()) return null;

        //clear image queue
        VisionDataEntry entry = visionProduct.pop();
        while(!visionProduct.isEmpty()){
            visionProduct.pop();
        }

        //if image is too old
        if(Timer.getFPGATimestamp() - entry.timestamp > 0.5) return null;

        Odometry.OldLocation oldLoc = r.sensors.odo.getOldRobotLocation(entry.timestamp);

        double minDist = 99999;
        VisionData bestData = null;
        for(VisionData d : entry.listFin){
            double dist = getHyp(d.pose.getTranslation());
            if(dist < minDist){
                minDist = dist;
                bestData = d;
            }
        }
        //SmartDashboard.putNumber("Min Vis Dist", minDist);
        //SmartDashboard.putString("Best Data", bestData.pose.toString());

        //Get tag location in field coordinates
        frc.robot.util.Vector cam = fromTranslation3dImage(bestData.pose.getTranslation());
        cam.add(camLocation);
        cam.theta += oldLoc.angle;
        cam.add(oldLoc.space);

        if(debug) System.out.println("Raw ID: " + bestData.tagId);
        int id = bestData.tagId;
        if(DriverStation.getAlliance() == Alliance.Red){
            id = 9 - id;
        }

        frc.robot.util.Vector tagPos = fromTranslation3dTag(AutonPos.tagLayout.getTagPose(id).get().getTranslation());
        if(debug) System.out.println("tag" + id + "Pos: " + tagPos.toStringXY());
        /*if(DriverStation.getAlliance() == Alliance.Blue)*/ position = 10 - position;
        if((level == 0 || position == 0) && scoring) return null;
        AutonPos offset;
        if(scoring){
            offset = new AutonPos(AutonPos.SCORING_OFFSETS[level - 1][position - 1]);
        } else {
            offset = new AutonPos(AutonPos.GATHER_OFFSET);
        }
        /*
        if(DriverStation.getAlliance() == Alliance.Red){
            offset.mirrorY(true);
            tagPos.theta = -tagPos.theta;
        }
        */
        frc.robot.util.Vector driveOffset = offset.xy.sub(tagPos);

        cam.add(driveOffset);
        System.out.println("Cam: " + cam);
        return cam;
    }

    public double getImageAngle(int level, int position){
        if(level == 0 || position == 0) return 0;
        return AutonPos.SCORING_OFFSETS[level - 1][position - 1].value;
    }

    public static frc.robot.util.Vector fromTranslation3dImage(Translation3d translation){
        return frc.robot.util.Vector.fromXY(Units.metersToInches(translation.getZ()), -Units.metersToInches(translation.getX()));
    }

    public static frc.robot.util.Vector fromTranslation3dTag(Translation3d translation){
        return frc.robot.util.Vector.fromXY(Units.metersToInches(translation.getX()), Units.metersToInches(translation.getY()));
    }

    public double getHyp(Translation3d translation){
        return Math.sqrt(translation.getX() * translation.getX() + 
                         translation.getY() * translation.getY() + 
                         translation.getZ() * translation.getZ());
    }
    
}

