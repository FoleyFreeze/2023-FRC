package frc.robot.subsystems.Sensors;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class VisionData {
    public byte tagId;
    public byte type;
    public Pose3d pose;
    public VisionData(byte type, byte tagId, double rotX, double rotY, double rotZ, double transX, double transY, double transZ){
        this.tagId = tagId;
        this.type = type;
        this.pose = new Pose3d(transX, transY, transZ, new Rotation3d(rotX, rotY, rotZ));
    }

}
