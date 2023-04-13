package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class ArmMove extends CommandBase{
 
    RobotContainer r;
    Vector position;
    PositionProvider provider;

    boolean instant = false;
    
    public ArmMove(RobotContainer r, Vector position, boolean instant){
        this(r,position);
        this.instant = instant;
    }

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
        if(instant) return true;
        
        Vector v = r.arm.getError();
        return (Math.abs(v.r) < 2.0) && Math.abs(v.theta) < Math.toRadians(3) ;
    }
}
