package frc.robot.util;

public class Angle {
    
    //TODO: Add this function to the drivetrain a bunch of times wherever we need to
    public static double normRad(double ang){
        if(ang > 2 * Math.PI){
            return ang - 2 * Math.PI;
        } else if(ang < 0){
            return ang + 2 * Math.PI;
        } else {
            return ang;
        }
    }

    public static double normDeg(double ang){
        if(ang > 360){
            return ang - 360;
        } else if(ang < 0){
            return ang + 360;
        } else {
            return ang;
        }
    }

    public static double tooRad(double ang){
        return (ang/360) * Math.PI / 2;
    }

    public static double toDeg(double ang){
        return (ang/(Math.PI/2)) * 360;
    }
}
