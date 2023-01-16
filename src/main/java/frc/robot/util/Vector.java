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
        double angle = Math.atan2(y, x);

        return new Vector(length, angle);
    }

    public static Vector addVectors(Vector v1, Vector v2){

        double x = v1.getX() + v2.getX();
        double y = v1.getY() + v2.getY();

        double length = Math.sqrt((x * x) + (y * y));
        double angle = Math.atan2(y, x);

        return new Vector(length, angle);
    }

    public String toStringXY(){
        return String.format("%.1f, %.1f", getX(), getY());
    }

    public String toStringPolar(){
        return String.format("%.1f, %.1f", r, theta);
    }

    @Override
    public String toString(){
        return toStringXY();
    }

    public Vector negate(){
        r = -r;
        return this;
    }

    public Vector add(Vector v){
        double x = getX() + v.getX();
        double y = getY() + v.getY();
        theta = Math.atan2(y,x);
        r = Math.sqrt(x*x + y*y);
        
        return this;
    }

    public double getX(){
        return r * Math.cos(theta);
    }

    public double getY(){
        return r * Math.sin(theta);
    }
}
