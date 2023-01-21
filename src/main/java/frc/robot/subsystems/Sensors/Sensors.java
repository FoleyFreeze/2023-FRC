package frc.robot.subsystems.Sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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
        odo = new Odometry(new OdometryCals(), r.dCal.wheelCals);
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

    public void resetNavXAng(){
        navX.reset();
        System.out.println("angle has been reset");
    }

    public void resetBotPos(){
        odo.setBotLocation(Vector.fromXY(0, 0));
        System.out.println("position has been reset");
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("Robot Pos: ", odo.botLocation.toStringXY());

        double robotYaw = navX.getYaw();
        Vector[] wheelStates = r.driveTrain.getWheelState();
        odo.update(robotYaw, wheelStates);
    }
}
