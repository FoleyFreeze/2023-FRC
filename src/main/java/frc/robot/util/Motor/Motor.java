package frc.robot.util.Motor;


public interface Motor {
    
    public static Motor create(MotorCal cal){
        switch(cal.type){
            case SPARK:
                return new SparkMotor(cal);
            default:
                return new NullMotor(cal);
        }
    }

    public abstract void setPower(double power);
    public abstract double getPosition();
    public abstract void setPosition(double position);
    public abstract void invert(boolean inverted);
}
