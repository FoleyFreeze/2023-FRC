package frc.robot.util.Motor;

public class NullMotor implements Motor{
    
    public NullMotor(MotorCal cal){
        
    }

    @Override
    public void setPower(double power) {
        
    }

    @Override
    public double getPosition() {
        return 0;
    }
    
    @Override
    public void setPosition(double position) {
        
    }

    @Override
    public void setEncoderPosition(double position) {
        
    }

    @Override
    public void setSpeed(double rpm) {
        
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public double getTemp(){
        return 0;
    }
}
