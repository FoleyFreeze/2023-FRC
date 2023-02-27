package frc.robot.subsystems.Gripper;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Motor.Motor;

public class Gripper extends SubsystemBase{
    
    RobotContainer r;
    public GripperCal cals; 

    Motor rGrip;
    Motor lGrip; 
    Servo choiceGrip;

    public Gripper (RobotContainer r, GripperCal cals){
        this.r = r;
        this.cals = cals;

        if(cals.disabled) return;

        rGrip = Motor.create(cals.rGrip);
        lGrip = Motor.create(cals.lGrip);
        choiceGrip = new Servo(cals.servoChannel);
    }

    //intake speed in rpm
    public void setIntakeSpeed(double rpm){
        rGrip.setSpeed(rpm);
        lGrip.setSpeed(-rpm);
    }

    //intake speed -1 to 1
    public void setIntakePower(double power){
        rGrip.setPower(power);
        lGrip.setPower(-power);
    }

    //open gripper
    public void open(){
        choiceGrip.set(cals.servoOpenPos);
        System.out.println("Open");
    }

    //close gripper
    public void close(){
        choiceGrip.set(cals.servoClosePos);
        System.out.println("Close");
    }

    //get electric current of both motors
    public double getIntakeCurrent(){
        return lGrip.getCurrent() + rGrip.getCurrent();
    }

    @Override
    public void periodic(){
    }
}
