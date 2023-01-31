package frc.robot.subsystems.Arm;

import frc.robot.util.Vector;
import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class ArmCal {
    
    final public boolean disabled = true;

    public MotorCal angleMotor = new MotorCal(MotorType.SPARK, 0).setRatio(360/10.0);
    public MotorCal lengthMotor = new MotorCal(MotorType.SPARK, 0);

    //max/min arm can move
    public double angleMax = 0;   
    public double angleMin = 0;
                                //TODO: get real values
    public double[] lengthMax = {2, 5, 7, 30, 30, 30};
    public double[] angleAxis = {0,25,60,100,175,180};
    public double lengthMin = 0;

    //common arm positions
    public Vector positionHome = new Vector(0, 0);
    
    public Vector positionPreGather = new Vector(0, 0);
    public Vector positionGatherShelf = new Vector(0, 0);
    public Vector positionGatherFloor = new Vector(0, 0);
    public Vector positionGatherFloorFar = new Vector(0, 0);

    //cone arm positions
    public Vector positionConeHiHold = new Vector(0, 0);
    public Vector positionConeHiRelease = new Vector(0, 0);
    public Vector positionConeMedHold = new Vector(0, 0);
    public Vector positionConeMedRelease = new Vector(0, 0);
    public Vector positionConeLowRelease = new Vector(0, 0);
  
    //cube arm positions
    public Vector positionCubeHi = new Vector(0, 0);
    public Vector positionCubeMed = new Vector(0, 0);
    public Vector positionCubeLow = new Vector(0, 0);
  

}
