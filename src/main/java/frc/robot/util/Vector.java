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

    public Vector(Vector v){
        r = v.r;
        theta = v.theta;
    }

    public static Vector fromXY(double x, double y){
        double length = Math.sqrt((x * x) + (y * y));
        double angle = Math.atan2(y, x);

        return new Vector(length, angle);
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

    public static Vector subVector(Vector v1, Vector v2){
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

    public String toStringXY(){
        return String.format("%.1f, %.1f", getX(), getY());
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
}
