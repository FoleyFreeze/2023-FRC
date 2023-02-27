package frc.robot.subsystems.Sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class Sensors extends SubsystemBase{
    
    RobotContainer r;
    SensorCal cal;

    public Odometry odo;

    public AHRS navX;
    private double navXOffset = 0;

    public double prevAng;
    public double prevWheelPos;

    public double dt;
    public double prevTime;

    public Sensors(RobotContainer r, SensorCal cal){
        this.r = r;
        this.cal = cal;

        navX = new AHRS();
        odo = new Odometry(new OdometryCals(), r.dCal.wheelCals);
    }

    public double getNavXAng(){
        return -(navX.getFusedHeading() + navXOffset) / 360.0 * Math.PI * 2;//returns in radians (-pi -> pi)
    }

    public void resetNavXAng(double ang){
        navX.reset();
        navXOffset = ang - navX.getFusedHeading();
        System.out.println("angle has been reset");
    }

    public void resetBotAng(){
        odo.botAngle = 0;
        resetNavXAng(0);
    }

    public void resetBotPos(){
        odo.setBotLocation(Vector.fromXY(0, 0));
        System.out.println("position has been reset");
    }

    double pitchOffset;
    double rollOffset;
    public void initPitchRoll(){
        pitchOffset = navX.getPitch();
        rollOffset = navX.getRoll();
    }

    public double getPitchRoll(){
        double pitch = navX.getPitch() - pitchOffset;
        double roll = navX.getRoll() - rollOffset;
        if (Math.abs(pitch) > Math.abs(roll)){ 
                return pitch;
        }else{
                return roll;
        }
    }

    public double getAbsPitchRoll(){
        double pitch = navX.getPitch() - pitchOffset;
        double roll = navX.getRoll() - rollOffset;
        if (Math.abs(pitch) > Math.abs(roll)){ 
                return Math.abs(pitch);
        }else{
                return Math.abs(roll);
        }
    }

    public double maxPitchRoll = 0;
    @Override
    public void periodic(){
        double now = Timer.getFPGATimestamp();
        dt = now - prevTime;
        prevTime = now;

        double robotYaw = getNavXAng();
        if(!r.driveTrain.cals.disabled) {
            Vector[] wheelStates = r.driveTrain.getWheelState();
            odo.update(robotYaw, wheelStates);
        }

        SmartDashboard.putNumber("Robot Ang: ", Math.toDegrees(odo.botAngle));
        SmartDashboard.putString("Robot Pos: ", odo.botLocation.toStringXY());

        SmartDashboard.putNumber("Wheel Slips", odo.badWheels);

        double temp = getAbsPitchRoll();
        SmartDashboard.putNumber("Roll/Pitch", temp);
        if(temp > maxPitchRoll) maxPitchRoll = temp;
        SmartDashboard.putNumber("MaxPitch/Roll", maxPitchRoll);
    }
}
