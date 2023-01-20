package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;
import java.lang.Math;

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

    //the distances are individual
    //but the times are cumulative
    private Vector totalDistance;
    private double startTime;
    private double decelTime;
    private double maxVelTime;
    private double accelTime;
    private double accelDist;
    private double decelDist;
    private double maxVelDist;
    public void initialize(){
        
        totalDistance =  Vector.subVector(endLoc, startLoc);
        double distThreshold = (AutonCal.maxVel * AutonCal.maxVel) / AutonCal.maxAccel;
       
        if (totalDistance.r >= distThreshold){
            //3 step (Accel - constV - Decel)
            accelDist = distThreshold / 2;
            decelDist = distThreshold / 2;
            maxVelDist = totalDistance.r - distThreshold;
            accelTime =  Math.sqrt(distThreshold / AutonCal.maxAccel);
            maxVelTime = accelTime + maxVelDist / AutonCal.maxVel;
            decelTime = maxVelTime + accelTime;
        } else{
            //2 step (Accel - Decel)
            maxVelTime = 0;
            maxVelDist = 0;
            accelDist = totalDistance.r / 2;
            decelDist = totalDistance.r / 2;
            accelTime =  Math.sqrt(totalDistance.r / AutonCal.maxAccel);
            decelTime = 2 * accelTime;
        }
        startTime = Timer.getFPGATimestamp();
    }
    
    public void execute (){
        double runTime = Timer.getFPGATimestamp() - startTime;
        
        Vector targetPos = new Vector(0,totalDistance.theta);
        Vector targetVel = new Vector(0,totalDistance.theta);
        double targetAccel;
        if (runTime < accelTime){ 
            //stage 1
            targetAccel = AutonCal.maxAccel;
            targetPos.r = 0.5 * targetAccel * runTime * runTime;
            targetVel.r = targetAccel * runTime;
        } else if (runTime < maxVelTime) { 
            //stage 2
            targetAccel = 0;
            targetVel.r = AutonCal.maxVel;
            targetPos.r = ((runTime - accelTime) * targetVel.r) + accelDist;
        } else if (runTime < decelTime) { 
            //stage 3
            double t = runTime - decelTime;
            targetAccel = -AutonCal.maxAccel;
            targetPos.r = 0.5 * targetAccel * t * t + totalDistance.r;
            targetVel.r = t * targetAccel;
        } else { 
            //end
            targetPos.r = totalDistance.r;
            targetVel.r = 0;
            targetAccel = 0;
        }
        
        //PID
        Vector currentPos = r.driveTrain.getPosition();
        targetPos.add(startLoc).negate().add(currentPos).negate();
        targetPos.r *= AutonCal.kP_MP;
        targetVel.add(targetPos);


        //output
        r.driveTrain.swerveMP(targetVel, targetAccel);
    }

    public boolean isFinished(){
        return false;
    }

    public void end(){

    }
}
