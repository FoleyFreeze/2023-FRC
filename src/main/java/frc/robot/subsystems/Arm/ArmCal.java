package frc.robot.subsystems.Arm;

import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class ArmCal {
    
    final public boolean disabled = true;

    public MotorCal angleMotor = new MotorCal(MotorType.SPARK, 0).setRatio(360/10.0);
    public MotorCal lengthMotor = new MotorCal(MotorType.SPARK, 0);

    public double angleMax = 0;   
    public double angleMin = 0;
                                //TODO: get real values
    public double[] lengthMax = {2, 5, 7, 30, 30, 30};
    public double[] angleAxis = {0,25,60,100,175,180};
    public double lengthMin = 0;

}
