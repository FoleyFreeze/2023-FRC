package frc.robot.subsystems.Drive;

import frc.robot.util.Vector;
import frc.robot.util.Motor.Motor;
import frc.robot.util.Motor.MotorCal;
import frc.robot.util.Motor.MotorCal.MotorType;

public class DriveCal {

    public class WheelCal{
        public MotorCal driveMotor;
        public MotorCal swerveMotor;
        public int encoderChannel;

        Vector wheelLocation;

        public final double swerveRotationsPerRev = 0;
        public final double driveRotationsPerRev = 0;
    }
    
    //TODO: Find motor channel inputs
    public WheelCal lFWheel = new WheelCal();{
        lFWheel.driveMotor = new MotorCal(MotorType.SPARK, 0);
        lFWheel.swerveMotor = new MotorCal(MotorType.SPARK, 0);

        lFWheel.wheelLocation = new Vector(0, 0);
    }

    public WheelCal rFWheel = new WheelCal();{
        rFWheel.driveMotor = new MotorCal(MotorType.SPARK, 0);
        rFWheel.swerveMotor = new MotorCal(MotorType.SPARK, 0);

        rFWheel.wheelLocation = new Vector(0, 0);
    }

    public WheelCal lRWheel = new WheelCal();{
        lRWheel.driveMotor = new MotorCal(MotorType.SPARK, 0);
        lRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 0);

        lRWheel.wheelLocation = new Vector(0, 0);
    }

    public WheelCal rRWheel = new WheelCal();{
        rRWheel.driveMotor = new MotorCal(MotorType.SPARK, 0);
        rRWheel.swerveMotor = new MotorCal(MotorType.SPARK, 0);

        rRWheel.wheelLocation = new Vector(0, 0);
    }

    public WheelCal[] wheelCals = {lFWheel, rFWheel, lRWheel, rRWheel};
}
