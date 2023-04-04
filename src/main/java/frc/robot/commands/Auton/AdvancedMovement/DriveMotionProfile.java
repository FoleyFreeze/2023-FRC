package frc.robot.commands.Auton.AdvancedMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.commands.Auton.AutonCal.MPCals;
import frc.robot.util.Angle;
import frc.robot.util.Vector;
import java.lang.Math;

public class DriveMotionProfile extends CommandBase{

    public RobotContainer r;
    private Vector startLoc;
    private Vector endLoc;
    private boolean threeStep; //threeStep is true when doing a three step profile, false when doing a two step
    final boolean DEBUG = false;
    double targetAngle;

    MPCals cals;

    public DriveMotionProfile(RobotContainer r, Vector endLoc, double targetAngle){
        this(r, endLoc, targetAngle, AutonCal.driveBase);
    }

    public DriveMotionProfile(RobotContainer r, Vector endLoc, double targetAngle, MPCals cals){
        this.r = r;
        this.endLoc = endLoc;
        this.targetAngle = targetAngle;
        this.cals = cals;
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
        double distThreshold = (cals.maxVel * cals.maxVel) / cals.maxAccel;
       
        System.out.format("DMP Starting at t:%.1f. Start: %s, End: %s\n",Timer.getFPGATimestamp(),startLoc.toString(),endLoc.toString());

        if (totalDistance.r >= distThreshold){
            //3 step (Accel - constV - Decel)
            accelDist = distThreshold / 2;
            decelDist = distThreshold / 2;
            maxVelDist = totalDistance.r - distThreshold;
            accelTime =  Math.sqrt(distThreshold / cals.maxAccel);
            //System.out.println("" + accelTime);
            maxVelTime = accelTime + maxVelDist / cals.maxVel;
            decelTime = maxVelTime + accelTime;
        } else{
            //2 step (Accel - Decel)
            maxVelTime = 0;
            maxVelDist = 0;
            accelDist = totalDistance.r / 2;
            decelDist = totalDistance.r / 2;
            accelTime =  Math.sqrt(totalDistance.r / cals.maxAccel);
            decelTime = 2 * accelTime;
        }
        startTime = Timer.getFPGATimestamp();
    }
    
    public void execute (){
        double runTime = Timer.getFPGATimestamp() - startTime;
        
        double[] state = getAVP(runTime);
        Vector targetPos = new Vector(state[0],totalDistance.theta);
        Vector targetVel = new Vector(state[1],totalDistance.theta);
        double targetAccel = state[2];
        
        //PID
        Vector currentPos = r.sensors.odo.botLocation;
        Vector distVec = new Vector(startLoc).sub(currentPos).add(targetPos);
        double errorMag = distVec.r;
        distVec.r *= cals.kP_MP;
        Vector totalVel = new Vector(targetVel).add(distVec);

        //output
        totalVel.r *= cals.kV;
        Vector power = new Vector(cals.kA * targetAccel, targetVel.theta).add(totalVel);
        power.r += cals.kS;

        //voltage compensation
        power.r *= 12.0 / r.lights.pdh.getVoltage();
        r.driveTrain.swerveMP(power,targetAngle);
        
        if(DEBUG) System.out.format("t:%.2f, p:%.2f, err:%.0f, x:%.0f, v:%.0f, a:%.0f, t1:%.1f, t2:%.1f, t3:%.1f\n", runTime,power.r,errorMag,targetPos.r,totalVel.r/cals.kV,targetAccel,accelTime,maxVelTime,decelTime);
        
    }

    public boolean isFinished(){
        double runTime = Timer.getFPGATimestamp() - startTime;
        return runTime > decelTime;
    }

    public void end(){
        r.driveTrain.driveSwerve(new Vector(0,0), 0);
        System.out.format("DMP Ending at t:%.1f.\n",Timer.getFPGATimestamp());
    }

    //position/velocity/acceleration
    double[] avp = new double[3]; 
    public double[] getAVP(double t){
        double targetAccel;
        double targetVel;
        double targetPos;
        if (t < accelTime){ 
            //stage 1
            targetAccel = cals.maxAccel;
            targetPos = 0.5 * targetAccel * t * t;
            targetVel = targetAccel * t;
        } else if (t < maxVelTime) { 
            //stage 2
            targetAccel = 0;
            targetVel = cals.maxVel;
            targetPos = ((t - accelTime) * targetVel) + accelDist;
        } else if (t < decelTime) { 
            //stage 3
            double t3 = t - decelTime;
            targetAccel = -cals.maxAccel;
            targetPos = 0.5 * targetAccel * t3 * t3 + totalDistance.r;
            targetVel = t3 * targetAccel;
        } else { 
            //end
            targetPos = totalDistance.r;
            targetVel = 0;
            targetAccel = 0;
        }

        avp[0] = targetPos;
        avp[1] = targetVel;
        avp[2] = targetAccel;
        return avp;
    }

    public double getTime(){
        return startTime + decelTime;
    }
}
