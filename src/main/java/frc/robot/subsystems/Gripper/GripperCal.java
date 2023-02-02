package frc.robot.subsystems.Gripper;

import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class GripperCal{
    
    final public boolean disabled = true;

    public MotorCal rGrip = new MotorCal(MotorType.SPARK, 0).setRatio(1);
    public MotorCal lGrip = new MotorCal(MotorType.SPARK, 0).setRatio(1);
    public int servoChannel = 0;
}
