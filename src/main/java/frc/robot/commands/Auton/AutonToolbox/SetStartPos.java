package frc.robot.commands.Auton.AutonToolbox;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal.AutonPos;

public class SetStartPos extends CommandBase{
    
    RobotContainer r;
    AutonPos startPos;

    public SetStartPos(RobotContainer r, AutonPos startPos){
        this.r = r;
        this.startPos = startPos;
    }

    @Override
    public void execute(){
        r.sensors.odo.setBotPose(startPos);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
