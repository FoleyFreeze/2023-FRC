package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CmdDrive extends CommandBase{
    
    RobotContainer r;

    public CmdDrive(RobotContainer r){
        this.r = r;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        r.driveTrain.driveSwerve(r.inputs.getJoystickX(), r.inputs.getJoystickY(), r.inputs.getJoystickZR());
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
