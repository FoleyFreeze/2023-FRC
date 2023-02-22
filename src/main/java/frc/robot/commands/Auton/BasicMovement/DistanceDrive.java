package frc.robot.commands.Auton.BasicMovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.util.Vector;

public class DistanceDrive extends CommandBase {
    
    RobotContainer r;
    Vector distance;

    
    Vector endPosition;

    public DistanceDrive(RobotContainer r, Vector distance){
        this.r = r;
        this.distance = distance;

        addRequirements(r.driveTrain);
    }

    public DistanceDrive(RobotContainer r, AutonPos pos){
        this.r = r;
        this.distance = pos.xy;

        addRequirements(r.driveTrain);
    }

    @Override
    public void initialize(){
        endPosition = Vector.addVectors(r.sensors.odo.botLocation, distance);
        prevDist = distance.r;
    }

    @Override
    public void execute(){
        Vector currPos = r.sensors.odo.botLocation;
        Vector direction = Vector.subVectors(endPosition, currPos);
        direction.r = 0.3;
        r.driveTrain.driveSwerve(direction, 0);
    }

    double prevDist;
    @Override
    public boolean isFinished(){
        Vector currPos = r.sensors.odo.botLocation;
        Vector distanceToTrgt = Vector.subVectors(endPosition, currPos);
        boolean done = distanceToTrgt.r > prevDist;
        prevDist = distanceToTrgt.r;
        return done;
    }
    
    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
    }
}
