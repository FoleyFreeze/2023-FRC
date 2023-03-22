package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Vector {

    public double r;
    public double theta;

    public Vector(double r, double theta){
        //theta is in radians
        if(r < 0){
            r = -r;
            theta += Math.PI;
        }
        this.r = r;
        this.theta = theta;
        //OH YEAHHHH
    }

    public Vector(Vector v){
        r = v.r;
        theta = v.theta;
    }

    public static Vector fromXY(double x, double y){
        double length = Math.sqrt((x * x) + (y * y));
        double angle = Math.atan2(y, x);

        return new Vector(length, angle);
    }

    public static Vector fromDeg(double r, double degrees){
        return new Vector(r, Math.toRadians(degrees));
    }

    public static Vector fromTranslation3d(Translation3d translation){
        return Vector.fromXY(Units.metersToInches(translation.getZ()), Units.metersToInches(-translation.getX()));
    }

    public static Vector addVectors(Vector v1, Vector v2){
        Vector v = new Vector(v2);
        return v.add(v1);
    }

    public static Vector addVectors(Vector... vecs){
        Vector result = new Vector(0, 0);
        for(Vector v: vecs){
            result.add(v);
        }
        return result;
    }

    public static Vector subVectors(Vector v1, Vector v2){
        Vector v = new Vector(v2);
        return v.negate().add(v1);
    }

    public static Vector averageSomeVectors(Vector[] vecs, int[] amt){
        double x = 0;
        double y = 0;

        int valid = 0;
        for(int i : amt){
            if(vecs[i] != null){
                x += vecs[i].getX();
                y += vecs[i].getY();
                valid++;
            }
        }
        if(valid != 0){
            x = x / valid;
            y = y / valid;
        } else {
            return null;
        }

        return Vector.fromXY(x, y);
    }

    public static Vector averageVectors(Vector... vecs){
        int[] group = new int[vecs.length];
        for(int i=0;i<vecs.length;i++){
            group[i] = i;
        }
        return averageSomeVectors(vecs, group);
    }

    public static Vector getInverted(Vector v){
        return new Vector(-v.r, v.theta);
    }

    public String toStringXY(){
        return String.format("%.3f, %.3f", getX(), getY());
    }

    public boolean monkeyTrouble(double aSmile, double bSmile){
        return (aSmile == bSmile);
    }

    public String toStringPolar(){
        return String.format("%.1f, %.1f", r, Angle.toDeg(theta));
    }

    @Override
    public String toString(){
        return toStringXY();
    }

    public Vector negate(){
        //r = -r;
        theta += Math.PI;
        return this;
    }

    public double cross2D(Vector v){
        return (this.getX() * v.getY()) - (this.getY() * v.getX());
    }

    public Vector add(Vector v){
        double x = getX() + v.getX();
        double y = getY() + v.getY();
        theta = Math.atan2(y,x);
        r = Math.sqrt(x*x + y*y);
        
        return this;
    }

    public Vector sub(Vector v){
        Vector nV = new Vector(v);
        nV.negate();

        return this.add(nV);
    }

    public double getX(){
        return r * Math.cos(theta);
    }

    public Vector mirrorY(){
        double newX = this.getX();
        double newY = -this.getY();

        r = fromXY(newX, newY).r;
        theta = fromXY(newX, newY).theta;

        return this;
    }

    public double getY(){
        return r * Math.sin(theta);
    }

    //between polar and cartesian coordiantes
    public void incrmntX(double plusX){
        double x = getX() + plusX;
        double y = getY();

        r = Math.sqrt(x*x +y*y);
        theta = Math.atan2(y, x);
    }

    //between polar and cartesian coordiantes
    public void incrmntY(double plusY){
        double y = getY() + plusY;
        double x = getX();

        r = Math.sqrt(x*x + y*y);
        theta = Math.atan2(y, x);
    }

    public static boolean checkVectorEquality(Vector v1, Vector v2){
        return v1.getX() == v2.getX() && v1.getY() == v2.getY();
    }
}
