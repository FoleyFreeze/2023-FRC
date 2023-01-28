package frc.robot.util.Motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
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

        PIDController.setP(cal.p);
        PIDController.setI(cal.i);
        PIDController.setD(cal.d);
        PIDController.setFF(cal.ff);

        PIDController.setOutputRange(-cal.pidLim,cal.pidLim);
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
    public void resetPosition(double position) {
        REVLibError err = encoder.setPosition(position / cal.gearRatio);
        if(!err.equals(REVLibError.kOk)){
            System.out.println("Error resetting wheel: " + err.toString());
        }
    }

}
