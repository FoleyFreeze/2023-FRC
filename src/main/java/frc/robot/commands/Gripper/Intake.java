package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Intake extends CommandBase{
    
    RobotContainer r;
    double startTime;
    public Intake (RobotContainer r){
        this.r = r;
    }

    public void initialize(){
        startTime = Timer.getFPGATimestamp();
    }

    public void execute(){
        if (r.inputs.isCube()){
            r.gripper.setIntakePower(r.gripper.cals.cubePickUpPower);
        }else{
            r.gripper.setIntakePower(r.gripper.cals.conePickUpPower);
        }
        
        if (r.inputs.isCube()){
            r.gripper.open();
        }else{
            r.gripper.close();
        }
        
    }

    @Override
    public void end(boolean interrupted){
        r.gripper.setIntakePower(0);

    }

    public boolean isFinished(){
        if ((Timer.getFPGATimestamp() - startTime) > 0.5){
            return false;
        }

        if(r.inputs.isCube()){
            return r.gripper.getIntakeCurrent() > r.gripper.cals.cubeStalllCurrent;
        }else{
            return r.gripper.getIntakeCurrent() > r.gripper.cals.coneStallCurrent; 
        }
    }


}


