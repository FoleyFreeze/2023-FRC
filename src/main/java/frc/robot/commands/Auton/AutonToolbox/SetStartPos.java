package frc.robot.commands.Auton.AutonToolbox;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonPos;

public class SetStartPos extends CommandBase{
    
    RobotContainer r;
    AutonPos pos;

    public SetStartPos(RobotContainer r, AutonPos pos){
        this.r = r;
        this.pos = pos;
    }

    @Override
    public void execute(){
        r.sensors.odo.setBotPose(pos);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
