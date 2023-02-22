package frc.robot.subsystems.Arm;

import frc.robot.util.Vector;
import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class ArmCal {
    
    final public boolean disabled = false;

    public MotorCal angleMotor = new MotorCal(MotorType.SPARK, 17).invert().setRatio(1 / 45.0 * 17 / 66.0 * 360).setCurrLim(20).setPIDPwrLim(0.1);
    public MotorCal lengthMotor = new MotorCal(MotorType.SPARK, 14).setRatio(1/20.0*3.0).setCurrLim(20).setPIDPwrLim(0.1);

    public int armPotChannel = 4;
    public double armPotOffset = 0;// in radians
    public double armPotSlope = 0; // in radians per volt

    //max/min arm can move
    public double angleMax = 135;
    public double angleMin = -5;
                                //TODO: get real values
    public double[] lengthMax = {31, 31, 38, 38};
    public double[] angleAxis = { 0, 30, 60,160};
    public double lengthMin = 31;
    public double initialStendoPosition = lengthMax[lengthMax.length-1];

    public double jogUpDist = 1; //inches
    public double jogOutDist = 1;

    //stendo max length restriction based on arm angle error
    public double[] stendoLengthMax = {38, lengthMin};
    public double[] armAngleErrorAxis = {10, 40};

    //stendo enc reset
    public double minStendoResetTime = 9999; //min time between resets
    public double stendoResetCurrent = 0; //current threshold for reset (motor side)
    public double stendoResetCurrentTime = 0; //time current must be high for

    //common arm positions
    public Vector positionHome = Vector.fromDeg(30, 0);
    
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
