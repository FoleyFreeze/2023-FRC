package frc.robot.subsystems.Arm;

import frc.robot.util.Vector;
import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class ArmCal {
    
    final public boolean disabled = false;

    public MotorCal angleMotor = new MotorCal(MotorType.SPARK, 17).invert().setRatio(1 / 45.0 * 17 / 66.0 * 360).setCurrLim(40).setPIDPwrLim(0.75).setPIDF(0.1, 0, 0.7, 0).setBrakeMode(true);
    public MotorCal lengthMotor = new MotorCal(MotorType.SPARK, 14).invert().setRatio(1/20.0*Math.PI).setCurrLim(15).setPIDPwrLim(0.95).setPIDF(0.2, 0, 0, 0).setBrakeMode(true);

    public int armPotChannel = 4;
    public double armPotOffset = 0;// in radians
    public double armPotSlope = 0; // in radians per volt

    //stendo angle offset
    public double stendoPulleyAngleOffset = 10;
    public double stendoPulleyLengthOffset = 1.9;

    //max/min arm can move
    public double angleMax = 135;
    public double angleMin = -11;
                                
    public double[] lengthMax = {40, 40, 40, 40};
    public double[] angleAxis = { 0, 30, 60,160};
    public double lengthMin = 31;
    public double initialStendoPosition = 34.2;

    public double jogUpDist = 1; //inches
    public double jogOutDist = 1;

    //stendo max length restriction based on arm angle error
    public double[] stendoLengthMax = {40, lengthMin};
    public double[] armAngleErrorAxis = {10, 40};

    //stendo enc reset
    public double minStendoResetTime = 9999; //min time between resets
    public double stendoResetCurrent = 0; //current threshold for reset (motor side)
    public double stendoResetCurrentTime = 0; //time current must be high for

    //common arm positions
    public Vector positionHome = Vector.fromDeg(32, -3);
    
    public Vector positionPreGather = new Vector(0, 0);
    public Vector positionGatherShelf = Vector.fromDeg(31.5, 94.2);
    public Vector positionCubeGatherFloor = Vector.fromDeg(37.2, 35.5);
    public Vector positionConeGatherFloor = Vector.fromDeg(38, 38);
    public Vector positionGatherFloorFar = new Vector(0, 0);

    //cone arm positions
    public Vector positionConeHiAngle = Vector.fromDeg(31, 115);
    public Vector positionConeHiStendo = Vector.fromDeg(31, -5.4);
    public Vector positionConeHiHold = Vector.fromDeg(39, 117);
    public Vector positionConeHiRelease = Vector.fromDeg(39, 100);
    public Vector positionConeMedHold = Vector.fromDeg(31, 93);
    public Vector positionConeMedRelease = Vector.fromDeg(31, 70);
    public Vector positionConeLowRelease = Vector.fromDeg(31, 30);
  
    //cube arm positions
    public Vector positionCubeHi = Vector.fromDeg(38, 100);
    public Vector positionCubeMed = Vector.fromDeg(29, 75);
    public Vector positionCubeLow = Vector.fromDeg(31, 25.0);
}
