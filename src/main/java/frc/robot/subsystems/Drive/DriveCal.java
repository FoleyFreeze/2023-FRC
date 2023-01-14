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

        public final double swerveRotationsPerRev = 0;
        public final double driveRotationsPerRev = 0;

        int idx;
    }
    
    //TODO: Find motor channel inputs
    public WheelCal FLWheel = new WheelCal();{
        FLWheel.driveMotor = new MotorCal(MotorType.SPARK, 0);
        FLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 1);

        FLWheel.wheelLocation = new Vector(14, 15);
        FLWheel.encoderChannel = 0;

        FLWheel.idx = 0;
    }

    public WheelCal FRWheel = new WheelCal();{
        FRWheel.driveMotor = new MotorCal(MotorType.SPARK, 2);
        FRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 3);

        FRWheel.wheelLocation = new Vector(14, -15);
        FRWheel.encoderChannel = 1;

        FRWheel.idx = 1;
    }

    public WheelCal RLWheel = new WheelCal();{
        RLWheel.driveMotor = new MotorCal(MotorType.SPARK, 4);
        RLWheel.swerveMotor = new MotorCal(MotorType.SPARK, 5);

        RLWheel.wheelLocation = new Vector(-14, -15);
        RLWheel.encoderChannel = 2;

        RLWheel.idx = 2;
    }

    public WheelCal RRWheel = new WheelCal();{
        RRWheel.driveMotor = new MotorCal(MotorType.SPARK, 6);
        RRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 7);

        RRWheel.wheelLocation = new Vector(-14, 15);
        RRWheel.encoderChannel = 3;

        RRWheel.idx = 3;
    }

    public WheelCal[] wheelCals = {FLWheel, FRWheel, RLWheel, RRWheel};
}
