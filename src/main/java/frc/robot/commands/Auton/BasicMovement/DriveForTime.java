package frc.robot.commands.Auton.BasicMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class DriveForTime extends CommandBase {
    
    RobotContainer r;
    Vector direction;
    double seconds;

    double startTime;

    public DriveForTime(RobotContainer r, Vector direction, double seconds){
        this.r = r;
        this.direction = direction;
        this.seconds = seconds;

        addRequirements(r.driveTrain);
    }

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
        System.out.println("driving");
    }

    @Override
    public void execute(){
        Vector xy = new Vector(direction);
        xy.theta -= r.sensors.odo.botAngle;
        r.driveTrain.driveSwerve(xy, 0);
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
