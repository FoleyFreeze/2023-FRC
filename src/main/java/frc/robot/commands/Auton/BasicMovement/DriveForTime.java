package frc.robot.commands.Auton.BasicMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class DriveForTime extends CommandBase {
    
    RobotContainer r;
    Vector direction;
    double angle;
    double seconds;

    double startTime;

    public DriveForTime(RobotContainer r, Vector direction, double seconds){
        this(r, direction, -9000, seconds);
    }

    public DriveForTime(RobotContainer r, Vector direction, double angle, double seconds){
        this.r = r;
        this.direction = direction;
        this.angle = angle;
        this.seconds = seconds;

        addRequirements(r.driveTrain);
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
        System.out.println("driving");
        if(angle == -9000){
            angle = r.sensors.odo.botAngle;
        }
    }

    @Override
    public void execute(){
        Vector xy = new Vector(direction);
        xy.theta -= r.sensors.odo.botAngle;

        double error = r.sensors.odo.botAngle % Math.PI;
                
        if(Math.abs(error) > Math.PI/2){
            if(error > 0) error -= Math.PI;
            else error += Math.PI;
        }

        double z = error * 0.2;
        if(z > 0.1) z = 0.2;
        if(z < -0.1) z = -0.2;
        
        r.driveTrain.driveSwerve(xy, z);
    }

    @Override
    public boolean isFinished(){
        return Timer.getFPGATimestamp() - startTime > seconds;
    }

    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
    }
}
