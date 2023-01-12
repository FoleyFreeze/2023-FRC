package frc.robot.util.Motor;

public class MotorCal {
    
    public enum MotorType {
        SPARK, NULL
    }

    public MotorType type;

    public int channel;

    public boolean inverted;

    public double p = 0;
    public double i = 0;
    public double d = 0;
    public double ff = 0;
    public double dFilt = 0;

    public MotorCal(MotorType type, int channel){
        this.type = type;
        this.channel = channel;
    }

    public MotorCal invert(){
        inverted = true;
        return this;
    }

    public MotorCal setPIDF(double p, double i, double d, double f){
        this.p = p;
        this.i = i;
        this.d = d;
        ff = f;
        return this;
    }
}
