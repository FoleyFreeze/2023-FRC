package frc.robot.subsystems.Sensors;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class VisionData {
    public byte tagId;
    public byte type;
    public byte eBits;
    public double decisionMargin;
    public Pose3d pose;

    public VisionData(byte type, byte tagId, byte eBits, double decisionMargin, double rotX, double rotY, double rotZ, double transX, double transY, double transZ){
        this.tagId = tagId;
        this.type = type;
        this.eBits = eBits;
        this.decisionMargin = decisionMargin;
        this.pose = new Pose3d(transX, transY, transZ, new Rotation3d(rotX, rotY, rotZ));
    }

    public VisionData(byte type, byte tagId, double rotX, double rotY, double rotZ, double transX, double transY, double transZ){
        this.tagId = tagId;
        this.type = type;
        this.pose = new Pose3d(transX, transY, transZ, new Rotation3d(rotX, rotY, rotZ));
    }

    public VisionData(byte type, double rotX, double rotY, double rotZ, double transX, double transY, double transZ){
        this.type = type;
        this.pose = new Pose3d(transX, transY, transZ, new Rotation3d(rotX, rotY, rotZ));
    }
}
