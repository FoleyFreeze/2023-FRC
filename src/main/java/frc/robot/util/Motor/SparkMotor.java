package frc.robot.util.Motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMotor implements Motor{
    
    MotorCal cal;
    CANSparkMax motor;

    RelativeEncoder encoder;
    SparkMaxPIDController PIDController;

    public SparkMotor(MotorCal cal){
        this.cal = cal;
        motor = new CANSparkMax(cal.channel, MotorType.kBrushless);

        motor.restoreFactoryDefaults();

        encoder = motor.getEncoder();
        PIDController = motor.getPIDController();

        motor.setInverted(cal.inverted);

        if(cal.rampRate != 0){
            motor.setOpenLoopRampRate(cal.rampRate);
            motor.setClosedLoopRampRate(cal.rampRate);
        }

        PIDController.setP(cal.p);
        PIDController.setI(cal.i);
        PIDController.setIZone(cal.iZone);
        PIDController.setD(cal.d);
        PIDController.setFF(cal.ff);
        PIDController.setDFilter(cal.dFilt);

        powerLimMax = cal.pidLimUp;
        powerLimMin = cal.pidLimDn;
        PIDController.setOutputRange(cal.pidLimDn,cal.pidLimUp);

        if(cal.currLim != 0){
            motor.setSmartCurrentLimit((int) cal.currLim);
            motor.setSecondaryCurrentLimit(cal.currLim);
        }

        if(cal.brakeMode){
            motor.setIdleMode(IdleMode.kBrake);
            brakeMode = true;
        } else {
            motor.setIdleMode(IdleMode.kCoast);
            brakeMode = false;
        }
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition() * cal.gearRatio;//in rotations
    }

    @Override
    public void setPosition(double position){
        PIDController.setReference(position / cal.gearRatio, ControlType.kPosition);//in rotations
    }

    @Override
    public void setEncoderPosition(double position) {
        REVLibError err = encoder.setPosition(position / cal.gearRatio);
        if(!err.equals(REVLibError.kOk)){
            System.out.println("Error resetting wheel: " + err.toString());
        }
    }

    @Override
    public void setSpeed(double rpm){
        PIDController.setReference(rpm / cal.gearRatio, ControlType.kVelocity);
    }

    @Override
    public double getCurrent(){
        return motor.getOutputCurrent();
    }

    @Override
    public double getTemp(){
        return motor.getMotorTemperature();
    }

    boolean brakeMode;
    @Override
    public void setBrakeMode(boolean brakeMode) {
        if(brakeMode != this.brakeMode){
            this.brakeMode = brakeMode;
            if(brakeMode){
                motor.setIdleMode(IdleMode.kBrake);
            } else {
                motor.setIdleMode(IdleMode.kCoast);
            }
        }
    }

    double powerLimMin,powerLimMax;
    @Override
    public void setPIDPwrLim(double pwrLim) {
        if(pwrLim != powerLimMax){
            PIDController.setOutputRange(-pwrLim,pwrLim);
            powerLimMax = pwrLim;
            powerLimMin = -pwrLim;
        }
    }
}
