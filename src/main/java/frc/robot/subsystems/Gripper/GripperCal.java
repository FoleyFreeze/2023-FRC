package frc.robot.subsystems.Gripper;

import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class GripperCal{
    
    final public boolean disabled = true;

    public MotorCal rGrip = new MotorCal(MotorType.SPARK, 0).setRatio(1).setCurrLim(10);
    public MotorCal lGrip = new MotorCal(MotorType.SPARK, 0).setRatio(1).setCurrLim(10);
    public int servoChannel = 0;

    public double cubePickUpPower = .3;
    public double cubePickUpSpeed = 2000;

    public double cubeScoreSpeed = 0;

    public double conePickUpPower = .4;
    public double conePickUpSpeed = 3000;

    public double coneScoreSpeed = 0;

    public double eject = -2000;

    public double coneStallCurrent = 30;
    public double cubeStalllCurrent = 30;
}
