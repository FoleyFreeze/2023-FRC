package frc.robot.commands.arm;

import frc.robot.RobotContainer;

public class ArmGoHome extends ArmMove{
    
    public ArmGoHome(RobotContainer r){
        super(r, r.arm.cals.positionHome);
    }

    //home position (angle motor only)
    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            r.arm.moveArmOnly(position);
        }
    }
} 
