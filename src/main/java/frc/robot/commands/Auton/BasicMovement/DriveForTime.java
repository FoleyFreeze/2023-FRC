package frc.robot.commands.Auton.BasicMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Angle;
import frc.robot.util.Util;
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
        this.angle = Angle.toRad(angle);
        this.seconds = seconds;

        addRequirements(r.driveTrain);
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        Vector xy = new Vector(direction);
        xy.theta -= r.sensors.odo.botAngle;

        double error = (angle - r.sensors.odo.botAngle) % (2 * Math.PI);

        SmartDashboard.putNumber("Error", Angle.toDeg(error));
                
        if(Math.abs(error) > Math.PI){
            if(error > 0) error -= 2 * Math.PI;
            else error += 2 * Math.PI;
        }

        double z = Util.bound(error * 0.3, -0.3, 0.3);
        
        if(angle == Angle.toRad(-9000)) z = 0;
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
