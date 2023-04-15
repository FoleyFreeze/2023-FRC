package frc.robot.subsystems.Drive;

import frc.robot.util.Vector;
import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class DriveCal {

    public final boolean disabled = false;

    public class WheelCal{
        //wheelcal contains all of the necessary information for each individual drive wheel to be set to
        
        public MotorCal driveMotor;
        public MotorCal swerveMotor;
        public int encoderChannel;

        public Vector wheelLocation;

        public final double swerveRotationsPerRev = 60.0;
        public final double driveRotationsPerIn = 64/18.0 * 18/32.0 * 45/15.0 / 4.0 / Math.PI;

        public final double driveInPerSwerveRotation = -32/18.0 * 15/45.0 * 4 * Math.PI;

        public double pidTime = 0.275;

        int idx;
        public String name;
    }
    
    double swerveKp = 0.4;//0.8;
    double swerveKi = 0.000001;
    double swerveIZone = 0.0;
    double swerveKd = 1.3;
    double swerveDfilt = 0.0;
    double swerveKf = 0;
    double swerveLim = 0.7;
    double swerveCurrLim = 45;
    double swerveRamp = 0.04;

    double driveCurrLim = 60;
    double driveRampRate = 0.4;

    public double resetAngleDelay = 3;

    public WheelCal FLWheel = new WheelCal();{
        FLWheel.driveMotor = new MotorCal(MotorType.SPARK, 6).invert().setRampRate(driveRampRate);
        FLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 7).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim).setRampRate(swerveRamp);

        FLWheel.wheelLocation = Vector.fromXY(9.75, 8.75);//12.5, 10.75);
        FLWheel.encoderChannel = 2;

        FLWheel.idx = 1;
        FLWheel.name = "FLWheel";
    }

    public WheelCal FRWheel = new WheelCal();{
        FRWheel.driveMotor = new MotorCal(MotorType.SPARK, 2).invert().setRampRate(driveRampRate);
        FRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 3).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim).setRampRate(swerveRamp);

        FRWheel.wheelLocation = Vector.fromXY(9.75, -8.75);//12.5, -10.75);
        FRWheel.encoderChannel = 1;

        FRWheel.idx = 0;
        FRWheel.name = "FRWheel";
    }

    public WheelCal RLWheel = new WheelCal();{
        RLWheel.driveMotor = new MotorCal(MotorType.SPARK, 8).invert().setRampRate(driveRampRate);
        RLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 9).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim).setRampRate(swerveRamp);

        RLWheel.wheelLocation = Vector.fromXY(-9.75, 8.75);//-12.5, 10.75);
        RLWheel.encoderChannel = 3;

        RLWheel.idx = 2;
        RLWheel.name = "RLWheel";
    }

    public WheelCal RRWheel = new WheelCal();{
        RRWheel.driveMotor = new MotorCal(MotorType.SPARK, 20).invert().setRampRate(driveRampRate);
        RRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 1).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim).setRampRate(swerveRamp);

        RRWheel.wheelLocation = Vector.fromXY(-9.75, -8.75);//-12.5, -10.75);
        RRWheel.encoderChannel = 0;

        RRWheel.idx = 3;
        RRWheel.name = "RRWheel";
    }

    public WheelCal[] wheelCals = {FRWheel, FLWheel, RLWheel, RRWheel};

    public double fieldModePwr = 0.95;
    public double pitModePwr = 0.3;
    public double inchModePwr = 0.4;
    public double scoringStrafePwr = 0.35;
    public double scoringRotPwr = 0.2;

    public double autoAngleWaitTime = 0.4;
    public double autoAlignWaitTime = 3.0;
    public double autoAlignKp = 0.4;
    public double autoAlignKi = 1.0;
    public double autoAlignKd = 0.01;
    public double autoAlignMaxPower = 0.2;

    public boolean autoAngleDuringCoast = true;
    public double autoCoastAngleMinVel = 12;
}
