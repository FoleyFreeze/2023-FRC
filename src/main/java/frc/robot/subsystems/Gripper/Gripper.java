package frc.robot.subsystems.Gripper;

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
        choiceGrip = new Servo(0);
    }

    public void setIntakeSpeed(double rpm){
        rGrip.setSpeed(rpm);
        lGrip.setSpeed(rpm);
    }

    public void setIntakePower(double power){
        rGrip.setPower(power);
        lGrip.setPower(power);
    }

    public void open(){
        choiceGrip.set(0);
    }

    public void close(){
        choiceGrip.set(1);
    }

    public double getIntakeCurrent(){
        return lGrip.getCurrent() + rGrip.getCurrent();
    }
}
