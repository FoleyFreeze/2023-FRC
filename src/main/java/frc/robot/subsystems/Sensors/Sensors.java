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
    private double navXOffset = 0;

    public double prevAng;
    public double prevWheelPos;

    public Sensors(RobotContainer r, SensorCal cal){
        this.r = r;
        this.cal = cal;

        navX = new AHRS();
        odo = new Odometry(new OdometryCals(), r.dCal.wheelCals);
    }

    public double getNavXAng(){
        return -(navX.getRoll() + navXOffset) / 360.0 * Math.PI * 2;//returns in radians (-pi -> pi)
    }

    public void resetNavXAng(double ang){
        navX.reset();
        navXOffset = ang;
        System.out.println("angle has been reset");
    }

    public void resetNavXAng(){
        resetNavXAng(0);
        odo.botAngle = 0;
    }

    public void resetBotPos(){
        odo.setBotLocation(Vector.fromXY(0, 0));
        System.out.println("position has been reset");
    }

    @Override
    public void periodic(){
        double robotYaw = getNavXAng();
        Vector[] wheelStates = r.driveTrain.getWheelState();
        odo.update(robotYaw, wheelStates);

        SmartDashboard.putNumber("Robot Ang: ", Math.toDegrees(odo.botAngle));
        SmartDashboard.putString("Robot Pos: ", odo.botLocation.toStringXY());

        SmartDashboard.putNumber("Wheel Slips", odo.badWheels);
    }
}
