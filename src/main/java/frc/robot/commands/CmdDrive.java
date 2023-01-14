package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class CmdDrive extends CommandBase{
    
    RobotContainer r;

    public CmdDrive(RobotContainer r){
        this.r = r;
        addRequirements(r.driveTrain);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        //x and y are flipped between because of the way our axes work; x is forward (0 degrees)
<<<<<<< HEAD
        Vector xy = r.inputs.getJoystickXY();
=======
        Vector xy = Vector.fromXY(-r.inputs.getJoystickY(), -r.inputs.getJoystickX());
>>>>>>> e1655b29cc68c35a2a03e1471a99cc7d139746a8
        r.driveTrain.driveSwerve(xy, r.inputs.getJoystickZR());
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
