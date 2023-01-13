package frc.robot.subsystems.Drive;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.DriveCal.WheelCal;
import frc.robot.util.Angle;
import frc.robot.util.Vector;
import frc.robot.util.Motor.Motor;

public class Wheel {

    WheelCal cal;

    Motor driveMotor;
    Motor swerveMotor;

    AnalogInput encoder;

    Vector driveVec;
    Vector wheelLocation;

    public Wheel(WheelCal cal){
        this.cal = cal;

        driveMotor = Motor.create(cal.driveMotor);
        swerveMotor = Motor.create(cal.swerveMotor);
        encoder = new AnalogInput(cal.encoderChannel);
    }

    //Uses the drive vector obtained from the drive command in DriveTrain
    public void drive(){
        double tgtAng = driveVec.theta;
        double currAng = encoder.getVoltage() * cal.swerveRotationsPerRev * 2 * Math.PI;

        //Make sure we go the shortest way
        if(Math.abs(Angle.normRad(Math.abs(driveVec.theta) - Math.abs(currAng))) > Math.PI / 2){
            tgtAng -= Math.PI;
            driveVec.r = -driveVec.r;
        }

        swerveMotor.setPosition(tgtAng);
        driveMotor.setPower(driveVec.r);
    }
}
