package frc.robot.commands.Auton.AdvancedMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.util.Vector;
import java.lang.Math;

public class DriveMotionProfile extends CommandBase{
    public RobotContainer r;
    private Vector startLoc;
    private Vector endLoc;
    private boolean threeStep; //threeStep is true when doing a three step profile, false when doing a two step

    public DriveMotionProfile(RobotContainer r, Vector endLoc){
        this.r = r;
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

        startLoc = new Vector(r.sensors.odo.botLocation);
        
        totalDistance = Vector.subVectors(endLoc, startLoc);
        double distThreshold = (AutonCal.maxVel * AutonCal.maxVel) / AutonCal.maxAccel;
       
        if (totalDistance.r >= distThreshold){
            //3 step (Accel - constV - Decel)
            accelDist = distThreshold / 2;
            decelDist = distThreshold / 2;
            maxVelDist = totalDistance.r - distThreshold;
            accelTime =  Math.sqrt(distThreshold / AutonCal.maxAccel);
            System.out.println("" + accelTime);
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
    
    enum Stages {NONE, STAGE_ONE, STAGE_TWO, STAGE_THREE};
    Stages stage = Stages.NONE;

    public void execute (){
        double runTime = Timer.getFPGATimestamp() - startTime;
        
        Vector targetPos = new Vector(0,totalDistance.theta);
        Vector targetVel = new Vector(0,totalDistance.theta);
        double targetAccel;
        if (runTime < accelTime){ 
            stage = Stages.STAGE_ONE;
            //stage 1
            targetAccel = AutonCal.maxAccel;
            targetPos.r = 0.5 * targetAccel * runTime * runTime;
            targetVel.r = targetAccel * runTime;
        } else if (runTime < maxVelTime) { 
            stage = Stages.STAGE_TWO;
            //stage 2
            targetAccel = 0;
            targetVel.r = AutonCal.maxVel;
            targetPos.r = ((runTime - accelTime) * targetVel.r) + accelDist;
        } else if (runTime < decelTime) { 
            stage = Stages.STAGE_THREE;
            //stage 3
            double t = runTime - decelTime;
            targetAccel = -AutonCal.maxAccel;
            targetPos.r = 0.5 * targetAccel * t * t + totalDistance.r;
            targetVel.r = t * targetAccel;
        } else { 
            stage = Stages.NONE;
            //end
            targetPos.r = totalDistance.r;
            targetVel.r = 0;
            targetAccel = 0;
        }

        
        //PID
        Vector currentPos = r.sensors.odo.botLocation;
        Vector distVec = new Vector(startLoc).sub(currentPos);
        double errorMag = targetPos.r - distVec.r;
        distVec.add(targetPos).r *= AutonCal.kP_MP;
        Vector totalVel = new Vector(targetVel).add(distVec);

        //output
        r.driveTrain.swerveMP(totalVel, targetAccel);
        
        System.out.format("t:%.2f, err:%.0f, x:%.0f, v:%.0f, a:%.0f, t1:%.1f, t2:%.1f, t3:%.1f\n", runTime,errorMag,targetPos.r,targetVel.r,targetAccel,accelTime,maxVelTime,decelTime);
        //SmartDashboard.putString("MP Stage", stage.toString());
    }

    public boolean isFinished(){
        return false;//TODO: this needs to stop!!!!!
    }

    public void end(){

    }
}
