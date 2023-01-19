package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class DriveMotionProfile extends CommandBase{
    public RobotContainer r;
    private Vector startLoc;
    private Vector endLoc; 
    private boolean threeStep; //threeStep is true when doing a three step profile, false when doing a two step

    public DriveMotionProfile(RobotContainer r, Vector startLoc, Vector endLoc){
        this.r = r;
        this.startLoc = startLoc;
        this.endLoc = endLoc;
        addRequirements(r.driveTrain);
    }

    public void initialize(){
        
        Vector distance =  Vector.subVector(endLoc, startLoc);
        double distThreshold = (AutonCal.maxVel * AutonCal.maxVel) / AutonCal.maxAccel;
        if (distance.r >= distThreshold){
            threeStep = true;
        } else{
            threeStep = false;
        }
    }

    private boolean twoStep;
    private double t1;
    private double t2;
    public void execute (){

    }

    public boolean isFinished(){
        return false;
    }

    public void end(){

    }
}
