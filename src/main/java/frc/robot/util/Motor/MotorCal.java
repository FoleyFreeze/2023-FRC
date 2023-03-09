package frc.robot.util.Motor;

public class MotorCal {
    
    public enum MotorType {
        SPARK, NULL
    }

    public MotorType type;

    public int channel;

    public boolean inverted = false;

    public double p = 0;
    public double i = 0;
    public double iZone = 0;
    public double d = 0;
    public double ff = 0;
    public double dFilt = 0;
    public double pidLimUp = 1;
    public double pidLimDn = -1;

    public double gearRatio = 1;

    public double currLim = 0;
    public double rampRate = 0;

    public boolean brakeMode = false;

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

    public MotorCal setPIDPwrLim(double lim){
        pidLimUp = lim;
        pidLimDn = -lim;
        return this;
    }

    public MotorCal setPIDPwrLim(double up, double dn){
        pidLimUp = up;
        pidLimDn = dn;
        return this;
    }

    public MotorCal setIZone(double iZone){
        this.iZone = iZone;
        return this;
    }

    public MotorCal setRatio(double ratio){
        gearRatio = ratio;
        return this;
    }

    public MotorCal setCurrLim(double lim){
        currLim = lim;
        return this;
    }

    public MotorCal setBrakeMode(boolean brakeMode){
        this.brakeMode = brakeMode;
        return this;
    }

    public MotorCal setRampRate(double rate){
        this.rampRate = rate;
        return this;
    }
}
