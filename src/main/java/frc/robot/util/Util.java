package frc.robot.util;

public class Util {
    
    public static double interp(double[] axis, double[] value, double x){
        if(x <= axis[0]){
            return value[0];
        } else if(x >= axis[axis.length-1]){
            return value[value.length-1];
        }
        
        int end = 1;
        for( ; end < axis.length; end++){
            if(x < end){
                break;
            }
        }

        double x1 = axis[end-1];
        double x2 = axis[end];
        double y1 = value[end-1];
        double y2 = value[end];
        return (y2-y1)/(x2-x1) * (x - x1) + y1;
    }

    public static double bound(double value, double min, double max){
        if (value < min)
            return min;
        if (value > max)
            return max;
        else return value;

    }


}
