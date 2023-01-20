package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ResetSwerveAngs extends CommandBase{

    RobotContainer r;

    double initTime;

    public ResetSwerveAngs(RobotContainer r){
        this.r = r;
    }

    @Override
    public void initialize(){
        initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted){
        r.driveTrain.writeAbsOffset();
    }

    @Override
    public boolean isFinished(){
        return initTime + r.driveTrain.cals.resetAngleDelay < Timer.getFPGATimestamp();
    }
}
