package frc.robot.util;

public class Angle {
    
    public static double normRad(double ang){
        ang %= 2 * Math.PI;
        if(ang > Math.PI){
            ang -= 2 * Math.PI;
        } else if(ang < -Math.PI){
            ang += 2 * Math.PI;
        }

        return ang;
    }

    public static double normDeg(double ang){
        ang %= 360;
        if(ang > 180){
            ang -= 360;
        } else if(ang < -180){
            ang += 360;
        }

        return ang;
    }

    public static double toRad(double ang){
        return (ang/360.0) * Math.PI * 2;
    }

    public static double toDeg(double ang){
        return (ang/Math.PI/2.0) * 360;
    }
}
