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

    boolean debug = true;

    //TODO: remove the added offset when the camera is recal'd
    frc.robot.util.Vector camLocation = frc.robot.util.Vector.fromXY(9.75, -8.75);

    private BooleanEntry active;
    private BooleanEntry tagsActive;
    private BooleanEntry cubesActive;
    private BooleanEntry conesActive; 
    private DoubleEntry rioTime;

    public LimitedStack<VisionDataEntry> tagVisionStack;
    private RawSubscriber poseMsgTag;
    private ByteBuffer poseDataTag;
    
    public LimitedStack<VisionDataEntry> cubeVisionStack;
    private RawSubscriber poseMsgCube;
    private ByteBuffer poseDataCube;

    public LimitedStack<VisionDataEntry> coneVisionStack;
    private RawSubscriber poseMsgCone;
    private ByteBuffer poseDataCone;
    

    public Vision(RobotContainer r) {
        super();
        this.r = r;
    }

    public void init() {
        rioTime = NetworkTableInstance.getDefault().getDoubleTopic("/Vision/RIO Time").getEntry(Timer.getFPGATimestamp());
        active = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Active").getEntry(true);
        tagsActive = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Tag Enable").getEntry(false);
        cubesActive = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Cube Enable").getEntry(false);
        conesActive = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Cone Enable").getEntry(false);
        //setTagMode(); //default to tags
        
        poseMsgTag = NetworkTableInstance.getDefault().getTable("Vision").getRawTopic("Tag Pose Data Bytes").subscribe("raw", null);
        tagVisionStack = new LimitedStack<VisionDataEntry>(5);
        NetworkTableInstance.getDefault().addListener(poseMsgTag, 
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                poseDataTag = ByteBuffer.wrap(event.valueData.value.getRaw());
                byte type = poseDataTag.get(12); // type 1 = tag, type 2 = cone, type 3 = cube
                byte numTags = poseDataTag.get(13);
                if(type == 1 && numTags <= 4){
                    VisionDataEntry e = new VisionDataEntry();
                    e.listFin = new Vector<VisionData>();
                    e.seqNum = poseDataTag.getInt(0);
                    double current = Timer.getFPGATimestamp();
                    e.timestamp = current - ((current -  poseDataTag.getFloat(4) + poseDataTag.getFloat(8)) / 2.0);
                    // added tag decision margin (1 float) and error bits (1 byte) to message
                    // this takes each tag struct from 25 bytes to 30 bytes
                    // old: for(int i = 0, b = 14; i < numTags; i++, b += 25){ 
                    for(int i = 0, b = 14; i < numTags; i++, b += 30) { 
                        
                        VisionData visionData = new VisionData(type, poseDataTag.get(b), 
                            poseDataTag.get(b+1), poseDataTag.getFloat(b+2), 
                            poseDataTag.getFloat(b+6), poseDataTag.getFloat(b+10), poseDataTag.getFloat(b+14), 
                            poseDataTag.getFloat(b+18), poseDataTag.getFloat(b+22), poseDataTag.getFloat(b+26));
                        e.listFin.add(visionData);

                    }
                    tagVisionStack.push(e);
                }

            });

        poseMsgCube = NetworkTableInstance.getDefault().getTable("Vision").getRawTopic("Cube Pose Data Bytes").subscribe("raw", null);
        cubeVisionStack = new LimitedStack<VisionDataEntry>(5);
        NetworkTableInstance.getDefault().addListener(poseMsgCube, 
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                poseDataCube = ByteBuffer.wrap(event.valueData.value.getRaw());
                byte type = poseDataCube.get(12); // type 1 = tag, type 2 = cone, type 3 = cube
                byte numTags = poseDataCube.get(13);
                if(type == 3 && numTags <= 4){
                    VisionDataEntry e = new VisionDataEntry();
                    e.listFin = new Vector<VisionData>();
                    e.seqNum = poseDataCube.getInt(0);
                    double current = Timer.getFPGATimestamp();
                    e.timestamp = current - ((current -  poseDataCube.getFloat(4) + poseDataCube.getFloat(8)) / 2.0);
                    
                    // cube only reports angle and distance
                    VisionData visionData = new VisionData(type, 0, Math.toRadians(poseDataCube.getFloat(14)), 0, 0, 0, poseDataCube.getFloat(18));
                    e.listFin.add(visionData);

                    cubeVisionStack.push(e);
                }

            });

        poseMsgCone = NetworkTableInstance.getDefault().getTable("Vision").getRawTopic("Cone Pose Data Bytes").subscribe("raw", null);
        coneVisionStack = new LimitedStack<VisionDataEntry>(5);
        NetworkTableInstance.getDefault().addListener(poseMsgCone, 
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> {
                poseDataCone = ByteBuffer.wrap(event.valueData.value.getRaw());
                byte type = poseDataCone.get(12); // type 1 = tag, type 2 = cone, type 3 = cube
                byte numTags = poseDataCone.get(13);
                if(type == 2 && numTags <= 4){
                    VisionDataEntry e = new VisionDataEntry();
                    e.listFin = new Vector<VisionData>();
                    e.seqNum = poseDataCone.getInt(0);
                    double current = Timer.getFPGATimestamp();
                    e.timestamp = current - ((current -  poseDataCone.getFloat(4) + poseDataCone.getFloat(8)) / 2.0);
                    for(int i = 0, b = 14; i < numTags; i++, b += 25){
                        
                        VisionData visionData = new VisionData(type, poseDataCone.get(b), 
                            Math.toRadians(poseDataCone.getFloat(b+1)), Math.toRadians(poseDataCone.getFloat(b+5)), Math.toRadians(poseDataCone.getFloat(b+9)), 
                            poseDataCone.getFloat(b+13), poseDataCone.getFloat(b+17), poseDataCone.getFloat(b+21));
                        e.listFin.add(visionData);
                    }
                    coneVisionStack.push(e);
                }

            });
    }
    @Override
    public void periodic() {

        rioTime.set(Timer.getFPGATimestamp());
        NetworkTableInstance.getDefault().flush();
    
    }

    public void setTagMode(){
        System.out.println("TagMode On");
        tagsActive.set(true);
        cubesActive.set(false);
        conesActive.set(false);
    }

    public void setCubeMode(){
        System.out.println("CubeMode On");
        tagsActive.set(false);
        cubesActive.set(true);
        conesActive.set(false);
    }

    public void setConeMode(){
        System.out.println("ConeMode On");
        tagsActive.set(false);
        cubesActive.set(false);
        conesActive.set(true);
    }

    public void allOff(){
        tagsActive.set(false);
        cubesActive.set(false);
        conesActive.set(false);
    }

    public void activate(boolean state) {

        // put the state (true or false) into the network table topic that will acivate or deactivate the Pi
        active.set(state);

    }


    public void togglePi(){
        boolean status = active.get();
        active.set(!status);
    }

    public frc.robot.util.Vector getCubeVector(){
        if(cubeVisionStack.isEmpty()) return null;

        VisionDataEntry vde = cubeVisionStack.pop();
        while(!cubeVisionStack.isEmpty()){
            cubeVisionStack.pop();
        }

        if(Timer.getFPGATimestamp() - vde.timestamp > 0.5) return null;

        Odometry.OldLocation oldLoc = r.sensors.odo.getOldRobotLocation(vde.timestamp);

        VisionData vd = vde.listFin.get(0);

        double dist = vd.pose.getZ();//to account for bumpers
        double ang = vd.pose.getRotation().getY() - Math.toRadians(2.0);//camera not mounted straight offset
        frc.robot.util.Vector cam = frc.robot.util.Vector.fromXY(dist,-dist*Math.tan(ang));
        if(debug) System.out.println("Raw Cam: " + cam.toStringXY());
        
        cam.add(frc.robot.util.Vector.fromXY(3.25, -8.75));//camera offset
        cam.add(frc.robot.util.Vector.fromXY(13.0,0.0));//robot length
        cam.theta += oldLoc.angle;
        cam.add(oldLoc.space);
        if(debug) System.out.println("Field Cam: " + cam.toStringXY());

        return cam;
    }

    public frc.robot.util.Vector getConeVector(){
        if(coneVisionStack.isEmpty()) return null;

        VisionDataEntry vde = coneVisionStack.pop();
        while(!coneVisionStack.isEmpty()){
            coneVisionStack.pop();
        }

        if(Timer.getFPGATimestamp() - vde.timestamp > 0.5) return null;

        Odometry.OldLocation oldLoc = r.sensors.odo.getOldRobotLocation(vde.timestamp);

        VisionData vd = vde.listFin.get(0);

        double dist = vd.pose.getZ();
        double ang = vd.pose.getRotation().getY();
        frc.robot.util.Vector cam = frc.robot.util.Vector.fromXY(dist,-dist*Math.tan(ang));
        if(debug) System.out.println("Raw Cam: " + cam.toStringXY());

        cam.add(frc.robot.util.Vector.fromXY(13.0,0.0));
        cam.theta += oldLoc.angle;
        cam.add(oldLoc.space);
        if(debug) System.out.println("Field Cam: " + cam.toStringXY());

        return cam;
    }

    int prevId = 0;
    public frc.robot.util.Vector getImageVector(int level, int position, boolean scoring){
        //if image doesnt exists
        if(tagVisionStack.isEmpty()) return null;

        //clear image queue
        VisionDataEntry entry = tagVisionStack.pop();
        //while(!tagVisionStack.isEmpty()){
        //    tagVisionStack.pop();
        //}
    
        tagVisionStack.clear(); //This replaces the above three lines to clear the vision stack - JC

        //if image is too old
        if(Timer.getFPGATimestamp() - entry.timestamp > 0.12) return null;

        Odometry.OldLocation oldLoc = r.sensors.odo.getOldRobotLocation(entry.timestamp);

        double minDist = 99999;
        VisionData bestData = null;
        for(VisionData d : entry.listFin){
            //ignore tags with wrong heights
            boolean good = false;
            double y = -Units.metersToInches(d.pose.getTranslation().getY());
            double x = Units.metersToInches(d.pose.getTranslation().getZ());
            switch(d.tagId){
                case 1:
                case 2:
                case 3:
                    good = scoring && DriverStation.getAlliance() == Alliance.Red && y > -5 && y < 5;
                break;
                case 6:
                case 7:
                case 8:
                    good = scoring && DriverStation.getAlliance() == Alliance.Blue && y > -5 && y < 5;
                break;
                case 4:
                    good = !scoring && DriverStation.getAlliance() == Alliance.Blue && y > 4 && y < 14;
                break;
                case 5:
                    good = !scoring && DriverStation.getAlliance() == Alliance.Red && y > 4 && y < 14;
                break;
                default:
                    good = false;
            }
            if(x < 0) {
                System.out.println("Image behind us: " + x);
                good = false;
            }
            if(d.eBits > 0){
                good = false;
            }
            
            if(good){
                //moved away from picking lowest hypoteneuse to lowest abs Y
                //this should prevent "bad" corner data from winning arbitration
                //double dist = getHyp(d.pose.getTranslation());
                double dist = Math.abs(d.pose.getTranslation().getX());
                if(dist < minDist){
                    minDist = dist;
                    bestData = d;
                }
            } else {
                System.out.println("Tag" + d.tagId + " rejected with y = " + String.format("%.0f",y) + " Err of: " + d.eBits + " DM: " + d.decisionMargin);
            }
        }
        if(bestData == null) return null;

        //SmartDashboard.putNumber("Min Vis Dist", minDist);
        //SmartDashboard.putString("Best Data", bestData.pose.toString());

        //Get tag location in field coordinates
        frc.robot.util.Vector cam = fromTranslation3dImage(bestData.pose.getTranslation());
        
        if(debug) System.out.println("Raw Cam: " + cam.toStringXY() + " Err of: " + bestData.eBits + " DM: " + bestData.decisionMargin);
        cam.add(camLocation);
        cam.theta += oldLoc.angle + Math.toRadians(2.0);//camera angle offset
        cam.add(oldLoc.space);
        if(debug) System.out.print("Field Cam: " + cam.toStringXY());
        if(debug) System.out.println("tdiff: " +(Timer.getFPGATimestamp() - entry.timestamp));

        if(debug || bestData.tagId != prevId) System.out.println("Raw ID: " + bestData.tagId);
        prevId = bestData.tagId;
        int id = bestData.tagId;
        if(DriverStation.getAlliance() == Alliance.Red){
            //ignore other teams tags
            if(id == 4 || id >= 6) return null;
            id = 9 - id;
        } else {
            //ignore other teams tags
            if(id <= 3 || id == 5) return null;
        }

        

        frc.robot.util.Vector tagPos = fromTranslation3dTag(AutonPos.tagLayout.getTagPose(id).get().getTranslation());
        if(debug) System.out.println("tag" + id + "Pos: " + tagPos.toStringXY());
        if(DriverStation.getAlliance() == Alliance.Blue) position = 10 - position;
        if((level == 0 || position == 0) && scoring) return null;
        AutonPos offset;
        if(scoring){
            offset = new AutonPos(AutonPos.SCORING_OFFSETS[level - 1][position - 1]);
        } else {
            offset = new AutonPos(AutonPos.GATHER_OFFSET);
        }
        
        if(DriverStation.getAlliance() == Alliance.Red){
            offset.mirrorY(true);
            //tagPos.mirrorY();
            tagPos.theta = -tagPos.theta;
        }
        
        frc.robot.util.Vector driveOffset = offset.xy.sub(tagPos);

        cam.add(driveOffset);
        if(debug) System.out.println("Cam: " + cam);
        return cam;
    }

    public frc.robot.util.Vector getTagForAuton(int tag){
        //if image doesnt exists
        if(tagVisionStack.isEmpty()) return null;

        //clear image queue
        VisionDataEntry entry = tagVisionStack.pop();
        //while(!tagVisionStack.isEmpty()){
        //    tagVisionStack.pop();
        //}
    
        tagVisionStack.clear(); //This replaces the above three lines to clear the vision stack - JC

        //if image is too old
        if(Timer.getFPGATimestamp() - entry.timestamp > 0.12) return null;

        Odometry.OldLocation oldLoc = r.sensors.odo.getOldRobotLocation(entry.timestamp);

        double minDist = 99999;
        VisionData bestData = null;
        for(VisionData d : entry.listFin){
            //ignore tags with wrong heights
            boolean good = false;
            double y = -Units.metersToInches(d.pose.getTranslation().getY());
            double x = Units.metersToInches(d.pose.getTranslation().getZ());
            switch(d.tagId){
                case 1:
                case 2:
                case 3:
                    good = DriverStation.getAlliance() == Alliance.Red && y > -5 && y < 5;
                break;
                case 6:
                case 7:
                case 8:
                    good = DriverStation.getAlliance() == Alliance.Blue && y > -5 && y < 5;
                break;
                case 4:
                    good = DriverStation.getAlliance() == Alliance.Blue && y > 4 && y < 14;
                break;
                case 5:
                    good = DriverStation.getAlliance() == Alliance.Red && y > 4 && y < 14;
                break;
                default:
                    good = false;
            }
            if(x < 0) {
                System.out.println("Image behind us: " + x);
                good = false;
            }
            if(d.eBits > 0){
                good = false;
            }
            if(d.tagId != tag){
                //ignore tags we are not looking for
                good = false;
            }
            
            if(good){
                //moved away from picking lowest hypoteneuse to lowest abs Y
                //this should prevent "bad" corner data from winning arbitration
                //double dist = getHyp(d.pose.getTranslation());
                double dist = Math.abs(d.pose.getTranslation().getX());
                if(dist < minDist){
                    minDist = dist;
                    bestData = d;
                }
            } else {
                System.out.println("Tag" + d.tagId + " rejected with y = " + String.format("%.0f",y) + " Err of: " + d.eBits + " DM: " + d.decisionMargin);
            }
        }
        if(bestData == null) return null;

        //SmartDashboard.putNumber("Min Vis Dist", minDist);
        //SmartDashboard.putString("Best Data", bestData.pose.toString());

        //Get tag location in field coordinates
        frc.robot.util.Vector cam = fromTranslation3dImage(bestData.pose.getTranslation());
        
        if(debug) System.out.println("Raw Cam: " + cam.toStringXY() + " Err of: " + bestData.eBits + " DM: " + bestData.decisionMargin);
        cam.add(camLocation);
        cam.theta += oldLoc.angle + Math.toRadians(2.0);//camera angle offset
        cam.add(oldLoc.space);
        if(debug) System.out.print("Field Cam: " + cam.toStringXY());
        if(debug) System.out.println("tdiff: " +(Timer.getFPGATimestamp() - entry.timestamp));

        if(debug || bestData.tagId != prevId) System.out.println("Raw ID: " + bestData.tagId);
        prevId = bestData.tagId;
        int id = bestData.tagId;
        if(DriverStation.getAlliance() == Alliance.Red){
            //ignore other teams tags
            if(id == 4 || id >= 6) return null;
            id = 9 - id;
        } else {
            //ignore other teams tags
            if(id <= 3 || id == 5) return null;
        }

        frc.robot.util.Vector tagPos = fromTranslation3dTag(AutonPos.tagLayout.getTagPose(id).get().getTranslation());
        if(debug) System.out.println("tag" + id + "Pos: " + tagPos.toStringXY());
        
        if(DriverStation.getAlliance() == Alliance.Red){
            tagPos.theta = -tagPos.theta;
        }

        cam.sub(tagPos);
        if(debug) System.out.println("Cam Delta: " + cam);
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

