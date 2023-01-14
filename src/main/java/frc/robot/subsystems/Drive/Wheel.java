package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
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

    WheelCal cal;

    Motor driveMotor;
    Motor swerveMotor;

    AnalogInput absEncoder;

    Vector driveVec;
    Vector wheelLocation;

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

    double setTime = 0;
    boolean inverted = false;
    //Uses the drive vector obtained from the drive command in DriveTrain
    public void drive(){
        double currAng = absEncoder.getVoltage() * cal.swerveRotationsPerRev * 2 * Math.PI;

        //Make sure we go the shortest way
        if(Math.abs(Angle.normRad(Math.abs(driveVec.theta) - Math.abs(currAng))) > Math.PI / 2){
            driveVec.theta -= Math.PI;
            if(inverted != true){
                driveVec.r = -driveVec.r;
                inverted = true;
            }
        } else {
            inverted = false;
        }

        swerveMotor.setPosition(driveVec.theta / (Math.PI * 2));
        driveMotor.setPower(driveVec.r);

        if(setTime < Timer.getFPGATimestamp()){
            setTime = Timer.getFPGATimestamp() + 2.0;
            System.out.println("Wheel: " + idx + "   x,y: " + driveVec.toString());
        }
    }
}
