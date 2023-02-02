package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Intake extends CommandBase{
    
    RobotContainer r;

    public Intake (RobotContainer r){
        this.r = r;
    }

    public void initialize(){

    }

    public void execute(){
        r.gripper.setIntakePower(.3);
    }

    public void end(){

    }

    public boolean ifFinished(boolean interrupted){
        return false;
    }

    }


