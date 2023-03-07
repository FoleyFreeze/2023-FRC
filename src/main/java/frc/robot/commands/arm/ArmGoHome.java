package frc.robot.commands.Arm;

import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class ArmGoHome extends ArmMove{
    
    public ArmGoHome(RobotContainer r){
        super(r, r.arm.cals.positionHome);
    }

    @Override
    public void execute(){
        r.arm.move(Vector.fromDeg(r.arm.cals.positionHome.r, r.arm.angleMotor.getPosition()));
    }

    //home position (angle motor only)
    @Override
    public void end(boolean interrupted){
        if(!interrupted){
            r.arm.move(position);
        }
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(r.arm.getError().r) < 3.0);
    }


} 
