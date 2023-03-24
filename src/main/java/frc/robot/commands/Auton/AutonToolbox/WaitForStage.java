package frc.robot.commands.Auton.AutonToolbox;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.DriveToImage;
import frc.robot.util.Vector;

public class WaitForStage extends CommandBase{

    double dist;
    DriveToImage dti;
    
    public WaitForStage(double dist, DriveToImage dti){
        this.dist = dist;
        this.dti = dti;
    }

    @Override
    public boolean isFinished(){
        System.out.println("FinishCheck " + Math.abs(dti.err.getY()));
        return dti.driveStage > 2 || 
              (dti.driveStage == 2 && Math.abs(dti.err.getY()) < dist);
    }
}
