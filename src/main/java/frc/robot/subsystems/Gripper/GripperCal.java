package frc.robot.subsystems.Gripper;

import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class GripperCal{
    
    final public boolean disabled = false;

    public MotorCal rGrip = new MotorCal(MotorType.SPARK, 15).setRatio(1).setCurrLim(20);
    public MotorCal lGrip = new MotorCal(MotorType.SPARK, 16).setRatio(1).setCurrLim(20);
    public int servoChannel = 1;

    public double servoOpenPos = 0.65;
    public double servoClosePos = 0.25;

    public double cubePickUpPower = .4;
    public double cubePickUpSpeed = 2000;

    public double cubeScoreSpeed = -4000;

    public double conePickUpPower = .4;
    public double conePickUpSpeed = 3000;

    public double coneScoreSpeed = -2000;

    public double eject = -2000;

    public double coneStallCurrent = 30;
    public double cubeStallCurrent = 30;
}
