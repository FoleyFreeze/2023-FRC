package frc.robot.subsystems.Drive;

import edu.wpi.first.networktables.GenericEntry;
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
        swerveMotor.setEncoderPosition(0.0);
        //subtract the offset from learned volt offsets; convert to 
        encAngOffset = (absEncoder.getVoltage() - voltOffset) / 5.0 * 2 * Math.PI - Math.PI / 2.0;
        System.out.println(cal.name + " angle offset: " + Angle.toDeg(encAngOffset));
        System.out.println(cal.name + "current voltage: " + absEncoder.getVoltage());
    }

    public void resetPosition(double offset){
        driveMotor.setEncoderPosition(offset);
    }

    double prevAngleSetpoint = 0;
    double angleTargetStartTime = 0;
    //Uses the drive vector obtained from the drive command in DriveTrain
    public void drive(boolean parkMode, boolean coasting){
        double rawRelEnc = swerveMotor.getPosition();
        double currAng = rawRelEnc / cal.swerveRotationsPerRev * 2 * Math.PI + encAngOffset;

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

        //if we are still targeting the same angle for cal time
        boolean allowRotate = true;
        if(Math.abs(targetRelEnc - prevAngleSetpoint) < 0.05){
            allowRotate = Timer.getFPGATimestamp() < angleTargetStartTime + cal.pidTime;
        } else {
            prevAngleSetpoint = targetRelEnc;
            angleTargetStartTime = Timer.getFPGATimestamp();
        }

        if(allowRotate || true){
            // if the wheel doesnt need to move, dont move it
            if(Math.abs(outputPower) > 0.03 || parkMode){
                swerveMotor.setPosition(targetRelEnc);
                if(coasting){
                    driveMotor.setPower(0);
                } else {
                    driveMotor.setPower(outputPower);
                }
            } else {
                driveMotor.setPower(0);
            }
        } else {
            swerveMotor.setPower(0);
            driveMotor.setPower(outputPower);
        }
        
    }

    public void driveAngleOnly(double angle){
        double rawRelEnc = swerveMotor.getPosition();
        double currAng = rawRelEnc / cal.swerveRotationsPerRev * 2 * Math.PI + encAngOffset;

        //Make sure we go the shortest way
        double delta = (angle - currAng) % (2 * Math.PI);
        if(delta < 0) delta += 2*Math.PI;
        //delta is now between 0-2pi

        if(delta > Math.PI){
            delta -= 2*Math.PI;
        }
        //delta is now between -pi-pi
        //System.out.println("delta: " + delta);

        double targetRelEnc = rawRelEnc + (delta / 2.0 / Math.PI * cal.swerveRotationsPerRev);
        //SmartDashboard.putNumber("Tgt Angle Enc", targetRelEnc);

        swerveMotor.setPosition(targetRelEnc);
    }

    //returns drive wheel's distance in inches
    public double getDist(){
        return getRawDrivePosition() / cal.driveRotationsPerIn + getRawSwervePosition() / cal.driveInPerSwerveRotation;
    }

    //returns the current motor angle in radians
    public double getAng(){
        return getRawSwervePosition() / cal.swerveRotationsPerRev * 2 * Math.PI + encAngOffset;
    }

    //cache the motor positions at the beginning of the periodic loop
    public void resetPosReads(){
        swerveRanThisTime = false;
        driveRanThisTime = false;
    }

    double swervePosition;
    boolean swerveRanThisTime;
    private double getRawSwervePosition(){
        if(swerveRanThisTime) {
            return swervePosition;
        } else {
            swervePosition = swerveMotor.getPosition();
            swerveRanThisTime = true;
            return swervePosition;
        }
    }

    double drivePosition;
    boolean driveRanThisTime;
    private double getRawDrivePosition(){
        if(driveRanThisTime){
            return drivePosition;
        } else {
            drivePosition = driveMotor.getPosition();
            driveRanThisTime = true;
            return drivePosition;
        }
    }
}
