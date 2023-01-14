package frc.robot.commands.Auton.BasicMovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
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

    @Override
    public void initialize(){
        endPosition = r.driveTrain.getPosition();
    }

    @Override
    public void execute(){
        Vector currPos = r.driveTrain.getPosition();
        Vector direction = Vector.addVectors(endPosition, currPos.negate());
        direction.r = 0.3;
        r.driveTrain.driveSwerve(direction, 0);
    }

    @Override
    public boolean isFinished(){
        Vector currPos = r.driveTrain.getPosition();
        Vector distanceToTrgt = Vector.addVectors(endPosition, currPos.negate());
        return distanceToTrgt.r < 4;
    }
    
    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
    }
}
