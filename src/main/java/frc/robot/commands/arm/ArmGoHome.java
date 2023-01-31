package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class ArmGoHome extends ArmMove{
    
    public ArmGoHome(RobotContainer r){
        super(r, r.arm.cals.positionHome);
    }

    //home position (angle only)
    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            r.arm.moveArmOnly(position);
        }
    }
} 
