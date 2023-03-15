package frc.robot.commands.Auton.BasicMovement;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.util.Angle;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class DistanceDrive extends CommandBase {
    
    RobotContainer r;

    Vector distance;
    double angle;
    
    Vector endPosition;

    public DistanceDrive(RobotContainer r, Vector distance){
        this(r, distance, -9000.0);
    }

    public DistanceDrive(RobotContainer r, Vector distance, double angle){
        this.r = r;
        this.distance = distance;
        this.angle = Math.toRadians(angle);

        addRequirements(r.driveTrain);
    }

    public DistanceDrive(RobotContainer r, AutonPos pos){
        this.r = r;
        this.distance = pos.xy;
        this.angle = Math.toRadians(pos.value);

        addRequirements(r.driveTrain);
    }

    @Override
    public void initialize(){
        endPosition = Vector.addVectors(r.sensors.odo.botLocation, distance);
        prevDist = distance.r;
    }

    @Override
    public void execute(){
        Vector currPos = new Vector(r.sensors.odo.botLocation);
        Vector direction = Vector.subVectors(endPosition, currPos);
        direction.r = 0.3;

        double error = (angle - r.sensors.odo.botAngle) % (2 * Math.PI);
                
        if(Math.abs(error) > Math.PI){
            if(error > 0) error -= 2 * Math.PI;
            else error += 2 * Math.PI;
        }

        double z = Util.bound(error * 0.3, -0.3, 0.3);
        
        if(angle == Angle.toRad(-9000)) z = 0;
        r.driveTrain.driveSwerve(direction, z);
    }

    double prevDist;
    @Override
    public boolean isFinished(){
        Vector currPos = new Vector(r.sensors.odo.botLocation);
        Vector distanceToTrgt = Vector.subVectors(endPosition, currPos);
        boolean done = Math.abs(distanceToTrgt.r - prevDist) > 3.0;
        prevDist = distanceToTrgt.r;
        return done;
    }
    
    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
    }
}
