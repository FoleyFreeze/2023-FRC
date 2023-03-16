package frc.robot.subsystems.Sensors;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableEvent;
import java.nio.ByteBuffer;
import java.util.EnumSet;
import java.util.Vector;
import java.util.concurrent.ConcurrentLinkedQueue;

public class Vision extends SubsystemBase {
    private BooleanEntry active;
    private DoubleEntry rioTime;
    private RawSubscriber poseMsg;
    private int listener;
    private ByteBuffer poseData;
    public ConcurrentLinkedQueue<VisionDataEntry> visionProduct;

    public Vision() {
        super();
    }

    public ConcurrentLinkedQueue<VisionDataEntry> init() {
        rioTime = NetworkTableInstance.getDefault().getDoubleTopic("/Vision/RIO Time").getEntry(Timer.getFPGATimestamp());
        active = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Active").getEntry(true);
        poseMsg = NetworkTableInstance.getDefault().getTable("Vision").getRawTopic("Pose Data Bytes").subscribe("raw", null);
        visionProduct = new ConcurrentLinkedQueue<VisionDataEntry>();
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
                    e.timestamp = current - ((current -  poseData.getFloat(4) - poseData.getFloat(8)) / 2);
                    for(int i = 0, b = 14; i < numTags; i++, b += 25){
                        
                        VisionData visionData = new VisionData(type, poseData.get(b), 
                            poseData.getFloat(b+1), poseData.getFloat(b+5), poseData.getFloat(b+9), 
                            poseData.getFloat(b+13), poseData.getFloat(b+17), poseData.getFloat(b+21));
                        e.listFin.add(visionData);
                    }
                    visionProduct.add(e);
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
    
}

