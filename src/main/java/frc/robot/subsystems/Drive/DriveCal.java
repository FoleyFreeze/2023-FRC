package frc.robot.subsystems.Drive;

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

    public double resetAngleDelay = 5;

    //TODO: Find motor channel inputs
    public WheelCal FLWheel = new WheelCal();{
        FLWheel.driveMotor = new MotorCal(MotorType.SPARK, 20);
        FLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 5).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setPIDPwrLim(swerveLim);

        FLWheel.wheelLocation = Vector.fromXY(12.5, 10.75);
        FLWheel.encoderChannel = 1;

        FLWheel.idx = 0;
        FLWheel.name = "FLWheel";
    }

    public WheelCal FRWheel = new WheelCal();{
        FRWheel.driveMotor = new MotorCal(MotorType.SPARK, 1);
        FRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 4).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setPIDPwrLim(swerveLim);

        FRWheel.wheelLocation = Vector.fromXY(12.5, -10.75);
        FRWheel.encoderChannel = 2;

        FRWheel.idx = 1;
        FRWheel.name = "FRWheel";
    }

    public WheelCal RLWheel = new WheelCal();{
        RLWheel.driveMotor = new MotorCal(MotorType.SPARK, 14);
        RLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 10).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setPIDPwrLim(swerveLim);

        RLWheel.wheelLocation = Vector.fromXY(-12.5, 10.75);
        RLWheel.encoderChannel = 0;

        RLWheel.idx = 2;
        RLWheel.name = "RLWheel";
    }

    public WheelCal RRWheel = new WheelCal();{
        RRWheel.driveMotor = new MotorCal(MotorType.SPARK, 15);
        RRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 11).setPIDF(swerveKp, swerveKi, swerveKd, swerveKf).setPIDPwrLim(swerveLim);

        RRWheel.wheelLocation = Vector.fromXY(-12.5, -10.75);
        RRWheel.encoderChannel = 3;

        RRWheel.idx = 3;
        RRWheel.name = "RRWheel";
    }

    public WheelCal[] wheelCals = {FLWheel, FRWheel, RLWheel, RRWheel};
}
