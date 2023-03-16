package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class ArmGoHome extends ArmMove{
    
    public ArmGoHome(RobotContainer r){
        super(r, r.arm.cals.positionHome);
    }

    @Override
    public void initialize(){
        //System.out.println("ArmGoHome started at time " + Timer.getFPGATimestamp());
    }

    @Override
    public void execute(){
        r.arm.move(Vector.fromDeg(r.arm.cals.positionHome.r, r.arm.angleMotor.getPosition()));
    }

    //home position (angle motor only)
    @Override
    public void end(boolean interrupted){
        System.out.println("ArmGoHome ended at time " + Timer.getFPGATimestamp());
        if(!interrupted){
            r.arm.move(position);
        }
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(r.arm.getError().r) < 3.0);
    }


} 
