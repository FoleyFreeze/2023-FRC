package frc.robot.commands.Auton.AutonToolbox;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.DriveToImage;
import frc.robot.util.Vector;

public class WaitForStage extends CommandBase{

    RobotContainer r;

    double dist;
    DriveToImage dti;
    boolean scoreMode;
    
    public WaitForStage(RobotContainer r, double dist, DriveToImage dti, boolean scoreMode){
        this.r = r;
        this.dist = dist;
        this.dti = dti;
        this.scoreMode = scoreMode;
    }

    @Override
    public boolean isFinished(){
        
        if(scoreMode){
            boolean ret = (dti.driveStage > 1 && dti.err.r < dist) 
            || (dti.driveStage == 1 && Math.abs(Vector.subVectors(dti.target, r.sensors.odo.botLocation).getY()) < dist);
            if(ret) System.out.println("DTI exit at stage: " + dti.driveStage + " dist: " + dti.err.getY());
            return ret;
        } else {
            return Math.abs(dti.angle - r.sensors.getNavXAng()) < Math.PI/10;
        }
    }
}
