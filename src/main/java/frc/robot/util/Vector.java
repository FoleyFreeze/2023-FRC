package frc.robot.util;

public class Vector {

    public double r;
    public double theta;

    public Vector(double r, double theta){
        //theta is in radians
        this.r = r;
        this.theta = theta;
        //OH YEAHHHH
    }

    public static Vector fromXY(double x, double y){
        double length = Math.sqrt((x * x) + (y * y));
        double angle = Math.atan(x/y);

        return new Vector(length, angle);
    }

    public static Vector addVectors(Vector v1, Vector v2){

        double x = (Math.cos(v1.theta) * v1.r) + (Math.cos(v2.theta) * v2.r);
        double y = (Math.sin(v1.theta) * v1.r) + (Math.sin(v2.theta) * v2.r);

        double length = Math.sqrt((x * x) + (y * y));
        double angle = Math.atan(x/y);

        return new Vector(length, angle);
    }
}
