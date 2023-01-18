package frc.robot.subsystems.Sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.DriveCal;
import frc.robot.subsystems.Drive.DriveCal.WheelCal;
import frc.robot.util.Vector;

public class Sensors extends SubsystemBase{
    
    RobotContainer r;
    SensorCal cal;

    public Odometry odo;

    public AHRS navX;

    public double prevAng;
    public double prevWheelPos;

    public Sensors(RobotContainer r, SensorCal cal){
        this.r = r;
        this.cal = cal;

        navX = new AHRS();
        odo = new Odometry(new OdometryCals(), r.driveTrain.cals);
    }

    public double getNavXAng(){
        return navX.getYaw() * 180 * Math.PI - 90;
    }

    public double getNavXPitch(){
        return navX.getPitch() * 180 * Math.PI;
    }

    public double getNavXRoll(){
        return navX.getRoll() * 180 * Math.PI;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("navX Yaw", navX.getYaw());
        SmartDashboard.putNumber("navX Pitch", navX.getPitch());
        SmartDashboard.putNumber("navX Roll", navX.getRoll());

        double robotYaw = navX.getYaw();
        Vector[] wheelStates = r.driveTrain.getWheelState();
        odo.update(robotYaw, wheelStates);
    }
}
