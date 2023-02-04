package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class ArmMove extends CommandBase{
 
    RobotContainer r;
    Vector position;
    PositionProvider provider;
    
    public ArmMove(RobotContainer r, Vector position){
        this.r = r;
        this.position = position;

        addRequirements(r.arm);
    }

    public ArmMove(RobotContainer r, PositionProvider provider){
        this.r = r;
        this.provider = provider;

        addRequirements(r.arm);
    }

    @Override
    public void initialize(){
        if(provider != null){
            position = provider.getPosition();
        }
    }


    //move arm to wanted position
    @Override
    public void execute(){
        r.arm.move(position);   
    }

    //stop if less than error
    @Override
    public boolean isFinished(){
        return (r.arm.getError() < 6);
    }
}
