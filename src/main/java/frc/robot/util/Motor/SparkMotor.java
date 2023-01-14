package frc.robot.util.Motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMotor implements Motor{
    
    CANSparkMax motor;

    RelativeEncoder encoder;
    SparkMaxPIDController PIDController;

    public SparkMotor(MotorCal cal){
        motor = new CANSparkMax(cal.channel, MotorType.kBrushless);
        encoder = motor.getEncoder();
        PIDController = motor.getPIDController();

        PIDController.setP(cal.p);
        PIDController.setI(cal.i);
        PIDController.setD(cal.d);
        PIDController.setFF(cal.ff);
    }

    @Override
    public void setPower(double power) {
        motor.set(power);
    }

    @Override
    public double getPosition() {
        return encoder.getPosition();//in rotations
    }

    @Override
    public void setPosition(double position){
        PIDController.setReference(position, ControlType.kPosition);//in rotations
    }

    @Override
    public void invert(boolean inverted) {
        motor.setInverted(inverted);
    }

}
