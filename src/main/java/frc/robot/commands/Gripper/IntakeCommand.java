package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeCommand extends CommandBase{
    
    RobotContainer r;
    double startTime;
    public IntakeCommand (RobotContainer r){
        this.r = r;
    }

    //start timer
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
    }

    //sets gripper power based on game piece; opens or closes gripper based on game piece
    public void execute(){
        if (r.inputs.isCube()){
            r.gripper.setIntakePower(r.gripper.cals.cubePickUpPower);
            r.gripper.open();
        }else{
            r.gripper.setIntakePower(r.gripper.cals.conePickUpPower);
            r.gripper.close();
        }
        
    }

    //intake power to 0
    @Override
    public void end(boolean interrupted){
        r.gripper.setIntakePower(0);

    }

    double stopTime = Double.POSITIVE_INFINITY;
    //false if current time minus start time is more less .5 sec; true if intake current is more than set stall current for game piece
    public boolean isFinished(){
        /*if ((Timer.getFPGATimestamp() - startTime) < 1.0){
            return false;
        }

        if(r.inputs.isCube()){
            if(r.gripper.getIntakeCurrent() > r.gripper.cals.cubeStallCurrent){
                stopTime = Timer.getFPGATimestamp() + 0.5;
            }
        }else{
            if(r.gripper.getIntakeCurrent() > r.gripper.cals.coneStallCurrent){
                stopTime = Timer.getFPGATimestamp() + 0.5;
                System.out.println("Stalled");
            }
        }

        if(Timer.getFPGATimestamp() > stopTime){
            return true;
        } else {
            return false;
        }*/
        return false;
    }


}


