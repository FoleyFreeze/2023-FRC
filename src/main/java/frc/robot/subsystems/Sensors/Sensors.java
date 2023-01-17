package frc.robot.subsystems.Sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Sensors extends SubsystemBase{
    
    RobotContainer r;
    SensorCal cal;

    public AHRS navX;

    public Sensors(RobotContainer r, SensorCal cal){
        this.r = r;
        this.cal = cal;

        navX = new AHRS();
    }

    public double getNavXAng(){
        return navX.getYaw() * 180 * Math.PI;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("navX Yaw", navX.getYaw());
        SmartDashboard.putNumber("navX Pitch", navX.getPitch());
        SmartDashboard.putNumber("navX Roll", navX.getRoll());
    }
}
