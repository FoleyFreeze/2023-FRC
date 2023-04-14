package frc.robot.commands.Auton.AdvancedMovement;

public class Tag{

    public enum Type {
        ANGLE,VEL,FLAG,VISION
    }

    double step;
    Type type;
    double value;

    public static Tag Angle(double step, double angle){
        Tag t = new Tag();
        t.type = Type.ANGLE;
        t.step = step;
        t.value = angle;
        return t;
    }

    public static Tag Vel(double step, double vel){
        Tag t = new Tag();
        t.type = Type.VEL;
        t.step = step;
        t.value = vel;
        return t;
    }

    public static Tag Flag(double step, double flag){
        Tag t = new Tag();
        t.type = Type.FLAG;
        t.step = step;
        t.value = flag;
        return t;
    }

    //-1 = cubes +X = looking for tag X (after red/blue flip)
    public static Tag Vision(double step, double vision){
        Tag t = new Tag();
        t.type = Type.VISION;
        t.step = step;
        t.value = vision;
        return t;
    }
}
