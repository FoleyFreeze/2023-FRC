package frc.robot.util.Motor;


public interface Motor {
    
    /* The motor class is designed to abstract away specific
     * motor conversion factors or functions that slow the
     * process of creating a subsystem. Instead of specific
     * sparks or talons being called in the code, an instance
     * of this class is used. It also allows the type of motor
     * to be changed universally and easily through the motor
     * type variable.
     */

    public static Motor create(MotorCal cal){
        switch(cal.type){
            case SPARK:
                return new SparkMotor(cal);
            default:
                return new NullMotor(cal);
        }
    }

    //Functions required for every motor to utiliize
    public abstract void setPower(double power);
    public abstract double getPosition();
    public abstract void setPosition(double position);
    public abstract void resetPosition(double position);
    public abstract void setSpeed(double rpm);
    public abstract double getCurrent();
}
