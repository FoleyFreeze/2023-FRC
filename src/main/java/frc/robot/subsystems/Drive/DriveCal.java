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

        int idx;
        public String name;
    }
    
    double swerveKp = 0.2;
    double swerveKi = 0.000;
    double swerveIZone = 1.0;
    double swerveKd = 0.1;
    double swerveKf = 0;
    double swerveLim = 0.5;
    double swerveCurrLim = 45;
    double driveCurrLim = 60;

    public double resetAngleDelay = 3;

    public WheelCal FLWheel = new WheelCal();{
        FLWheel.driveMotor = new MotorCal(MotorType.SPARK, 6).invert().setRampRate(0.8);
        FLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 7).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim);

        FLWheel.wheelLocation = Vector.fromXY(9.75, 8.75);//12.5, 10.75);
        FLWheel.encoderChannel = 2;

        FLWheel.idx = 2;
        FLWheel.name = "FLWheel";
    }

    public WheelCal FRWheel = new WheelCal();{
        FRWheel.driveMotor = new MotorCal(MotorType.SPARK, 2).invert().setRampRate(0.4);
        FRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 3).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim);

        FRWheel.wheelLocation = Vector.fromXY(9.75, -8.75);//12.5, -10.75);
        FRWheel.encoderChannel = 1;

        FRWheel.idx = 0;
        FRWheel.name = "FRWheel";
    }

    public WheelCal RLWheel = new WheelCal();{
        RLWheel.driveMotor = new MotorCal(MotorType.SPARK, 8).invert().setRampRate(0.4);
        RLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 9).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim);

        RLWheel.wheelLocation = Vector.fromXY(-9.75, 8.75);//-12.5, 10.75);
        RLWheel.encoderChannel = 3;

        RLWheel.idx = 3;
        RLWheel.name = "RLWheel";
    }

    public WheelCal RRWheel = new WheelCal();{
        RRWheel.driveMotor = new MotorCal(MotorType.SPARK, 20).invert().setRampRate(0.4);
        RRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 1).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setIZone(swerveIZone).setPIDPwrLim(swerveLim).setCurrLim(swerveCurrLim);

        RRWheel.wheelLocation = Vector.fromXY(-9.75, -8.75);//-12.5, -10.75);
        RRWheel.encoderChannel = 0;

        RRWheel.idx = 3;
        RRWheel.name = "RRWheel";
    }

    public WheelCal[] wheelCals = {FRWheel, FLWheel, RLWheel, RRWheel};

    public double fieldModePwr = 0.85;
    public double pitModePwr = 0.3;
    public double inchModePwr = 0.4;
    public double scoringStrafePwr = 0.35;
    public double scoringRotPwr = 0.2;

    public double autoAlignWaitTime = 1;
    public double autoAlignKp = -0.3;
    public double autoAlignMaxPower = 0.2;
}
