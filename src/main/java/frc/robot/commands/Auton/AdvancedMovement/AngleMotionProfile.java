package frc.robot.commands.Auton.AdvancedMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.util.Angle;
import frc.robot.util.Vector;
import java.lang.Math;

public class AngleMotionProfile extends CommandBase{
    public RobotContainer r;
    private double startLoc;
    private double endLoc;
    private boolean threeStep; //threeStep is true when doing a three step profile, false when doing a two step
    boolean DEBUG = false;


    public AngleMotionProfile(RobotContainer r, double endAngle){
        this.r = r;
        this.endLoc = endAngle;
        addRequirements(r.driveTrain);
    }

    //the distances are individual
    //but the times are cumulative
    private double totalDistance;
    private double startTime;
    private double decelTime;
    private double maxVelTime;
    private double accelTime;
    private double accelDist;
    private double decelDist;
    private double maxVelDist;
    boolean inverted = false;
    public void initialize(){

        startLoc = r.sensors.odo.botAngle;

        System.out.format("AMP Starting at t:%.1f. Start: %.1f, End: %.1f\n",Timer.getFPGATimestamp(),startLoc,endLoc);
        
        double totalAngle = Angle.shortestPath(startLoc, endLoc); 
        totalDistance = totalAngle * r.dCal.wheelCals[0].wheelLocation.r;
        double distThreshold = (AutonCal.AmaxVel * AutonCal.AmaxVel) / AutonCal.AmaxAccel;

        if(totalDistance < 0){
            inverted = true;
            totalDistance = -totalDistance;
        }

        if (totalDistance >= distThreshold){
            //3 step (Accel - constV - Decel)
            accelDist = distThreshold / 2;
            decelDist = distThreshold / 2;
            maxVelDist = totalDistance - distThreshold;
            accelTime =  Math.sqrt(distThreshold / AutonCal.AmaxAccel);
            //System.out.println("" + accelTime);
            maxVelTime = accelTime + maxVelDist / AutonCal.AmaxVel;
            decelTime = maxVelTime + accelTime;
        } else{
            //2 step (Accel - Decel)
            maxVelTime = 0;
            maxVelDist = 0;
            accelDist = totalDistance / 2;
            decelDist = totalDistance / 2;
            accelTime =  Math.sqrt(totalDistance / AutonCal.AmaxAccel);
            decelTime = 2 * accelTime;
        }
        startTime = Timer.getFPGATimestamp();
    }
    
    enum Stages {NONE, STAGE_ONE, STAGE_TWO, STAGE_THREE};
    Stages stage = Stages.NONE;

    public void execute (){
        double runTime = Timer.getFPGATimestamp() - startTime;
        
        double[] state = getAVP(runTime);
        double targetPos = state[0];
        double targetVel = state[1];
        double targetAccel = state[2];

        if(inverted){
            targetAccel *= -1;
            targetVel *= -1;
            targetPos *= -1;
        }

        //PID
        double currentPos = r.sensors.odo.botAngle;
        double delta = Angle.shortestPath(startLoc, currentPos);
        double targetAngle = targetPos / r.dCal.wheelCals[0].wheelLocation.r;
        double dist = (targetAngle - delta) * r.dCal.wheelCals[0].wheelLocation.r;
        double errorMag = dist;
        dist *= AutonCal.AkP_MP;
        double totalVel = targetVel + dist;

        //output
        double power = (AutonCal.AkA * targetAccel) + (AutonCal.AkV * totalVel) + AutonCal.AkS;
        r.driveTrain.swerveMPA(power);
        
        if(DEBUG) System.out.format("t:%.2f, p:%.2f, err:%.0f, x:%.0f, v:%.0f, a:%.0f, t1:%.1f, t2:%.1f, t3:%.1f\n", runTime,power,errorMag,targetPos,targetVel,targetAccel,accelTime,maxVelTime,decelTime);
    }

    public boolean isFinished(){
        double runTime = Timer.getFPGATimestamp() - startTime;
        return runTime > decelTime;
    }

    public void end(){
        r.driveTrain.driveSwerve(new Vector(0,0), 0);
        System.out.format("AMP Ending at t:%.1f.\n",Timer.getFPGATimestamp());
    }

    //position/velocity/position
    double[] avp = new double[3]; 
    public double[] getAVP(double t){
        double targetAccel;
        double targetVel;
        double targetPos;
        if (t < accelTime){ 
            //stage 1
            targetAccel = AutonCal.AmaxAccel;
            targetPos = 0.5 * targetAccel * t * t;
            targetVel = targetAccel * t;
        } else if (t < maxVelTime) { 
            //stage 2
            targetAccel = 0;
            targetVel = AutonCal.AmaxVel;
            targetPos = ((t - accelTime) * targetVel) + accelDist;
        } else if (t < decelTime) { 
            //stage 3
            double t3 = t - decelTime;
            targetAccel = -AutonCal.AmaxAccel;
            targetPos = 0.5 * targetAccel * t3 * t3 + totalDistance;
            targetVel = t3 * targetAccel;
        } else { 
            //end
            targetPos = totalDistance;
            targetVel = 0;
            targetAccel = 0;
        }

        avp[0] = targetPos;
        avp[1] = targetVel;
        avp[2] = targetAccel;
        return avp;
    }
}
