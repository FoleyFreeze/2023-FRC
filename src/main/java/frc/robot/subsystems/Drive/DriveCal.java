package frc.robot.subsystems.Drive;

import java.security.PublicKey;

import frc.robot.util.Vector;
import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class DriveCal {

    public class WheelCal{
        //wheelcal contains all of the necessary information for each individual drive wheel to be set to
        
        public MotorCal driveMotor;
        public MotorCal swerveMotor;
        public int encoderChannel;

        public Vector wheelLocation;

        public double drivePwr = 0.3;

        public final double swerveRotationsPerRev = 60.0;
        public final double driveRotationsPerIn = 64/18.0 * 18/32.0 * 45/15.0 / 4.0 / Math.PI;

        public final double driveInPerSwerveRotation = -32/18.0 * 15/45.0 * 4 * Math.PI;

        int idx;
        public String name;
    }
    
    double swerveKp = 0.1;
    double swerveKi = 0.000;
    double swerveKd = 0.0;
    double swerveKf = 0;
    double swerveLim = 0.5;

    public double resetAngleDelay = 3;

    //TODO: Find motor channel inputs
    public WheelCal FLWheel = new WheelCal();{
        FLWheel.driveMotor = new MotorCal(MotorType.SPARK, 20).invert();
        FLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 5).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setPIDPwrLim(swerveLim).setCurrLim(20);

        FLWheel.wheelLocation = Vector.fromXY(12.5, 10.75);
        FLWheel.encoderChannel = 2;

        FLWheel.idx = 1;
        FLWheel.name = "FLWheel";
    }

    public WheelCal FRWheel = new WheelCal();{
        FRWheel.driveMotor = new MotorCal(MotorType.SPARK, 1).invert();
        FRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 4).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setPIDPwrLim(swerveLim).setCurrLim(20);

        FRWheel.wheelLocation = Vector.fromXY(12.5, -10.75);
        FRWheel.encoderChannel = 1;

        FRWheel.idx = 0;
        FRWheel.name = "FRWheel";
    }

    public WheelCal RLWheel = new WheelCal();{
        RLWheel.driveMotor = new MotorCal(MotorType.SPARK, 14).invert();
        RLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 10).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setPIDPwrLim(swerveLim).setCurrLim(20);

        RLWheel.wheelLocation = Vector.fromXY(-12.5, 10.75);
        RLWheel.encoderChannel = 3;

        RLWheel.idx = 2;
        RLWheel.name = "RLWheel";
    }

    public WheelCal RRWheel = new WheelCal();{
        RRWheel.driveMotor = new MotorCal(MotorType.SPARK, 15).invert();
        RRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 11).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setPIDPwrLim(swerveLim).setCurrLim(20);

        RRWheel.wheelLocation = Vector.fromXY(-12.5, -10.75);
        RRWheel.encoderChannel = 0;

        RRWheel.idx = 3;
        RRWheel.name = "RRWheel";
    }

    public WheelCal[] wheelCals = {FRWheel, FLWheel, RLWheel, RRWheel};



    public double autoAlignWaitTime = 1;
    public double autoAlignKp = -0.3;
    public double autoAlignMaxPower = 0.2;
}
