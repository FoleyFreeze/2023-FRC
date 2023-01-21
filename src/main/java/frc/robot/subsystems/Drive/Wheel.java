package frc.robot.subsystems.Drive;

import java.beans.beancontext.BeanContextChildSupport;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive.DriveCal.WheelCal;
import frc.robot.util.Angle;
import frc.robot.util.Vector;
import frc.robot.util.Motor.Motor;

public class Wheel {

    /* The wheel class contains everything that each of the four 
     * drivetrain wheels need: a drive motor, swerve motor, and
     * encoder. It also uses information based on location to directly
     * send commands to the wheel motors to be driven using the
     * drive function.
     */

    public WheelCal cal;

    Motor driveMotor;
    Motor swerveMotor;

    AnalogInput absEncoder;

    Vector driveVec;
    Vector wheelLocation;

    public double encAngOffset;

    int idx;

    public Wheel(WheelCal cal){
        this.cal = cal;

        driveMotor = Motor.create(cal.driveMotor);
        swerveMotor = Motor.create(cal.swerveMotor);
        absEncoder = new AnalogInput(cal.encoderChannel);

        driveVec = new Vector(0, 0);
        wheelLocation = cal.wheelLocation;

        idx = cal.idx;
    }

    public void setEncAngOffset(double voltOffset){
        swerveMotor.resetPosition(0.0);
        encAngOffset = (absEncoder.getVoltage() - voltOffset) / 5.0 * 2 * Math.PI - Math.PI / 2.0;
        System.out.println(cal.name + " angle offset: " + Angle.toDeg(encAngOffset));
    }

    //Uses the drive vector obtained from the drive command in DriveTrain
    public void drive(){
        double rawRelEnc = swerveMotor.getPosition();
        double currAng = rawRelEnc / cal.swerveRotationsPerRev * 2 * Math.PI + encAngOffset;
        SmartDashboard.putNumber(cal.name + " currAng", Angle.toDeg(currAng));
        SmartDashboard.putNumber(cal.name + " rawEnc", rawRelEnc);

        //Make sure we go the shortest way
        double delta = (driveVec.theta - currAng) % (2 * Math.PI);
        if(delta < 0) delta += 2*Math.PI;
        //delta is now between 0-2pi

        if(delta > Math.PI){
            delta -= 2*Math.PI;
        }
        //delta is now between -pi-pi

        //flip r if delta is greater than pi/2
        double outputPower = driveVec.r;
        if(delta > Math.PI/2){
            delta -= Math.PI;
            outputPower = -driveVec.r;
        } else if(delta < -Math.PI/2){
            delta += Math.PI;
            outputPower = -driveVec.r;
        }

        double targetRelEnc = rawRelEnc + (delta / 2.0 / Math.PI * cal.swerveRotationsPerRev);

        // if the wheel doesnt need to move, dont move it
        if(outputPower != 0){
            swerveMotor.setPosition(targetRelEnc);
            driveMotor.setPower(outputPower * cal.drivePwr);
        } else {
            driveMotor.setPower(0);
        }

        SmartDashboard.putNumber(cal.name + " angle", Angle.toDeg(driveVec.theta));
        SmartDashboard.putNumber(cal.name + " delta", Angle.toDeg(delta));
        SmartDashboard.putNumber(cal.name + " power", outputPower);
    }

    //returns drive wheel's distance in inches
    public double getDist(){
        return driveMotor.getPosition() / cal.driveRotationsPerIn + swerveMotor.getPosition() / cal.driveInPerSwerveRotation;
    }

    //returns the current motor angle in radians
    public double getAng(){
        return swerveMotor.getPosition() / cal.swerveRotationsPerRev * 2 * Math.PI;
    }
}
