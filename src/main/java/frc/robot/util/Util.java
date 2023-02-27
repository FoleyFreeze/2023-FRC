package frc.robot.util;

public class Util {
    
    /*interp: interpolate
     draw a line between all possible points and then go to the line*/
    public static double interp(double[] axis, double[] value, double x){
        //if we are out of bounds return the bound
        if(x <= axis[0]){
            return value[0];
        } else if(x >= axis[axis.length-1]){
            return value[value.length-1];
        }
        
        //find the points we are between
        int end = 1;
        while(end < axis.length-1 && x > axis[end]){
            end++;
        }

        //create the line and find the new point on it
        double x1 = axis[end-1];
        double x2 = axis[end];
        double y1 = value[end-1];
        double y2 = value[end];
        return (y2-y1)/(x2-x1) * (x - x1) + y1;
    }

    //If value is above maximum/minimum then function returns the set maximum/minimum respectively
    public static double bound(double value, double min, double max){
        if (value < min)
            return min;
        if (value > max)
            return max;
        else return value;

    }


}
