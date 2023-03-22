package frc.robot.subsystems.Arm;

import frc.robot.util.Vector;
import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class ArmCal {
    
    final public boolean disabled = false;

    //                                                                                       used to be 17/66
    public MotorCal angleMotor = new MotorCal(MotorType.SPARK, 17).invert().setRatio(1 / 45.0 * 18 / 72.0 * 360).setCurrLim(40).setPIDPwrLim(0.75).setPIDF(0.1, 0, 0.7, 0).setBrakeMode(true);
    public MotorCal lengthMotor = new MotorCal(MotorType.SPARK, 14).invert().setRatio(1/20.0*Math.PI).setCurrLim(25).setPIDPwrLim(0.95).setPIDF(0.2, 0, 0, 0).setBrakeMode(true);

    public int armPotChannel = 4;
    public double armPotOffset = 0.968;// in volts
    public double armPotSlope = 1.121; // in radians per volt

    //stendo angle offset
    public double stendoPulleyAngleOffset = 10;
    public double stendoPulleyLengthOffset = 1.9;

    //max/min arm can move
    public double angleMax = 135;
    public double angleMin = -11;
                                
    public double[] lengthMax = {40, 40, 40, 40};
    public double[] angleAxis = { 0, 30, 60,160};
    public double lengthMin = 30;
    public double initialStendoPosition = 33.4;

    public double jogUpDist = 1; //inches
    public double jogOutDist = 1;//deg

    //stendo max length restriction based on arm angle error
    public double[] stendoLengthMax = {40, lengthMin};
    public double[] armAngleErrorAxis = {10, 40};

    //stendo enc reset
    public double minStendoResetTime = 9999; //min time between resets
    public double stendoResetCurrent = 0; //current threshold for reset (motor side)
    public double stendoResetCurrentTime = 0; //time current must be high for

    //common arm positions
    public Vector positionHome = Vector.fromDeg(30.65, -2);//(30.65, -5);
    
    //public Vector positionPreGather = Vector.fromDeg(0, 0);
    public Vector positionGatherShelf = Vector.fromDeg(33, 92.6);
    public Vector positionCubeGatherFloor = Vector.fromDeg(38, 36.8);
    public Vector positionConeGatherFloor = Vector.fromDeg(39.2, 40.5);

    //cone arm positions
    public Vector positionConeHiAngle = Vector.fromDeg(31, 110.7);
    public Vector positionConeHiStendo = Vector.fromDeg(31, -5.4);
    public Vector positionConeHiHold = Vector.fromDeg(39, 110.7);
    public Vector positionConeHiRelease = Vector.fromDeg(39, 97.3);
    public Vector positionConeMedHold = Vector.fromDeg(33.5, 89.3);
    public Vector positionConeMedRelease = Vector.fromDeg(33.5, 72.6);
    public Vector positionConeLowRelease = Vector.fromDeg(30.6, 19.8);
  
    //cube arm positions
    public Vector positionCubeHi = Vector.fromDeg(35, 102.3);
    public Vector positionCubeMed = Vector.fromDeg(30.6, 74.5);
    public Vector positionCubeLow = Vector.fromDeg(30.6, 25.8);
}
