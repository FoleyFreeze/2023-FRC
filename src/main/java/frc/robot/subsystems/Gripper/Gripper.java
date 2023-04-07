package frc.robot.subsystems.Gripper;

import com.ctre.phoenix.CANifier.PWMChannel;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Motor.Motor;

public class Gripper extends SubsystemBase{
    
    RobotContainer r;
    public GripperCal cals; 

    public Motor rGrip;
    public Motor lGrip;

    Servo choiceGrip;
    double servoDisableTime;

    GenericEntry lGripTempNT = Shuffleboard.getTab("Safety").add("lGrip Temp", 0).getEntry();
    GenericEntry rGripTempNT = Shuffleboard.getTab("Safety").add("rGrip Temp", 0).getEntry();

    GenericEntry maxGripCurrNT = Shuffleboard.getTab("Safety").add("Grip Curr", 0).getEntry();

    public Gripper (RobotContainer r, GripperCal cals){
        this.r = r;
        this.cals = cals;

        if(cals.disabled) return;

        rGrip = Motor.create(cals.rGrip);
        lGrip = Motor.create(cals.lGrip);
        choiceGrip = new Servo(cals.servoChannel);

        servoDisableTime = 0;
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
        choiceGrip.setDisabled();
        servoDisabled = false;
        choiceGrip.set(cals.servoOpenPos);
        servoDisableTime = Timer.getFPGATimestamp() + 0.5;
    }

    //close gripper
    public void close(){
        choiceGrip.setDisabled();
        servoDisabled = false;
        choiceGrip.set(cals.servoClosePos);
        servoDisableTime = Timer.getFPGATimestamp() + 0.5;
    }

    //get electric current of both motors
    public double getIntakeCurrent(){
        return lGrip.getCurrent() + rGrip.getCurrent();
    }

    double maxGripperCurrent = 0;
    boolean servoDisabled = false;
    @Override
    public void periodic(){
        if(cals.disabled == true) return;

        SmartDashboard.putNumber("lGrip Current", lGrip.getCurrent());
        SmartDashboard.putNumber("rGrip Current", rGrip.getCurrent());

        if(Timer.getFPGATimestamp() > servoDisableTime && servoDisabled == false){
            choiceGrip.setDisabled();
            servoDisabled = true;
        }

        if(getIntakeCurrent() > maxGripperCurrent){
            maxGripperCurrent = getIntakeCurrent();
        }
        maxGripCurrNT.setDouble(maxGripperCurrent);

        lGripTempNT.setDouble(lGrip.getTemp());
        rGripTempNT.setDouble(rGrip.getTemp());
    }
}
