package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AutonCal.MPCals;
import frc.robot.subsystems.Inputs.Inputs.Level;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class DriveToImageMP extends CommandBase{
    
    RobotContainer r;

    boolean scoreMode;

    MPCals mpCals;
    
    public int driveStage;
    public Vector err;

    boolean debug = true;

    double maxFilterDist = 1.0;//inches
    double maxSingleFrameOffset = 3;//inches
    double filterDivisor = 4.0;

    public DriveToImageMP(RobotContainer r, boolean scoreMode, MPCals mpCals){
        this.r = r;
        this.scoreMode = scoreMode;
        this.mpCals = mpCals;
    }

    @Override
    public void initialize(){
        r.vision.setTagMode();
        
        target = null;
        pwrMultiplier = 0.1;
        pwrMax = 0.3;

        driveStage = 0;
        err = new Vector(0,0);

        level = r.inputs.selectedLevel.ordinal();

        motionProfiling = false;

        System.out.println("DriveToImageMP Init. Score: " + scoreMode);
    }

    public Vector target;

    double pwrMultiplier;
    double pwrMax;
    final double PWR_MAX_X_MOVE = 0.5;
    final double PWR_MAX_CUBE = 0.3;
    final double PWR_MAX_CONE = 0.2;
    final double PWR_MAX_GATHER = 0.45;

    public double angle;

    public int level;
    public int position;

    boolean motionProfiling;
    Vector mpPwr;
    double mpCompletionTime;

    Vector mpStartLoc;
    double mpStartVel;
    double mpStartTime;

    double throwAwayThreshold = 225;
    double stage1ShelfXTarget = 0;

    double lastImageError = 0;
    final double stage1ErrThresh = 24;

    @Override
    public void execute(){
        SmartDashboard.putBoolean("MotionProfiling", motionProfiling);

        position = (r.inputs.selectedZone.ordinal() - 1) * 3 + r.inputs.selectedPosition.ordinal();
        
        if(target == null){
            //no active target
            Vector v = r.vision.getImageVector(level, position, scoreMode);
            if(v != null) {
                if(Vector.subVectors(v, r.sensors.odo.botLocation).r < throwAwayThreshold){
                    target = new Vector(v);
                    if(debug) System.out.print("Raw: " + v.toString());
                } else {
                    if(debug) System.out.print("Threw away raw: " + v.toString());
                }
                lastImageError = 0;
            }
        } else {
            //update existing target
            Vector newImage = r.vision.getImageVector(level, position, scoreMode);
            if(newImage != null){
                if(debug) System.out.print("Raw: " + newImage.toString());
                
                if(Vector.subVectors(newImage, r.sensors.odo.botLocation).r < throwAwayThreshold){
                    Vector delta = Vector.subVectors(newImage, target);
                    lastImageError = delta.r;
                    if(delta.r > maxFilterDist){
                        delta.r /= filterDivisor;
                        if(delta.r > maxSingleFrameOffset){
                            delta.r = maxSingleFrameOffset;
                        }
                        target.add(delta);
                    }
                } else {
                    if(debug) System.out.print("Threw away raw: " + newImage.toString());
                }
            }
        }



        Vector power;
        if(target == null){
            //no target yet
            if(scoreMode){
                angle = Math.PI;
            } else {
                angle = 0;
            }
            power = getJoystickPower();

        } else {
            //target exists
            if(debug) System.out.println(" Target: " + target.toString());

            //different logic based on if you're gathering or scoring
            if(scoreMode){

                //no else cases, so when we move to next stage we
                //immediately take the new drive action of that stage
                double strafeXOffset = 0;
                //for high cones offset extra towards the charge station
                if(!r.inputs.isCube() && r.inputs.selectedLevel == Level.TOP) strafeXOffset = 5;
                //for mid cones offset extra away from the charge station
                if(!r.inputs.isCube() && r.inputs.selectedLevel == Level.MIDDLE) strafeXOffset = -10;
                //for low scores offset extra away from the charge station
                if(r.inputs.selectedLevel == Level.BOTTOM) strafeXOffset = -10;
                
                if(driveStage == 0){
                    driveStage = 1;
                }
                if(driveStage == 1){
                    pwrMultiplier = 0.6;
                    pwrMax = PWR_MAX_X_MOVE;
                    //Move it to a mid-substation x position first
                    Vector xOffset = Vector.fromXY(target.getX() + AutonPos.tagToMidX + strafeXOffset, r.sensors.odo.botLocation.getY());
                    err = Vector.subVectors(xOffset, r.sensors.odo.botLocation);
                    angle = Math.PI;
                    if(Math.abs(err.getX()) < 4.0){
                        if(lastImageError < stage1ErrThresh){
                            driveStage = 2;
                            mpStartLoc = new Vector(r.sensors.odo.botLocation);
                            mpStartVel = 0;
                            mpStartTime = Timer.getFPGATimestamp();
                            if(Math.abs(new Vector(target).sub(r.sensors.odo.botLocation).getY()) > 10.0){
                                setMotionProfiling(true);
                            } else {
                                setMotionProfiling(false);
                            }
                        } else {
                            //new images dont agree with old target
                            driveStage = 0;
                            target = null;
                            System.out.println("Returning to stage 0 due to error of: " + lastImageError);
                        }
                    }
                } 
                if(driveStage == 2){
                    
                    pwrMultiplier = 0.35;
                    pwrMax = PWR_MAX_CUBE;
                    //Move it to the correct y position next
                    Vector yAlign = Vector.fromXY(target.getX() + AutonPos.tagToMidX + strafeXOffset, target.getY());
                    
                    err = Vector.subVectors(yAlign, r.sensors.odo.botLocation);
                    angle = r.vision.getImageAngle(level, position);
                    if(motionProfiling == true){
                        //Motion Profile
                        mpPwr = getMPPwr(mpStartLoc, mpStartVel, mpStartTime, yAlign);
                        if(Timer.getFPGATimestamp() - mpStartTime > mpCompletionTime){
                            driveStage = 3;
                            setMotionProfiling(false);
                        }
                    } else {
                        if((err.r < 1.0) || (err.r < 6.0 && level == 1 && r.inputs.isCube()) 
                                         || (err.r < 4.0 && level == 1 && !r.inputs.isCube())){
                            driveStage = 3;
                        }
                    }
                }
                if(driveStage == 3) {
                    pwrMultiplier = 0.3;
                    
                    //Final drive in
                    err = Vector.subVectors(target,r.sensors.odo.botLocation);
                    angle = r.vision.getImageAngle(level, position);
                    if(level == 1){
                        pwrMax = 0.4;
                    } else if(err.r < 40 && !r.inputs.isCube()){
                        pwrMax = PWR_MAX_CONE;
                    } else {
                        pwrMax = PWR_MAX_CUBE;
                    }
                    double angError = Angle.normRad(r.sensors.odo.botAngle - angle);
                    if(Math.abs(err.getX()) < 2.0 && Math.abs(err.getY()) < 1.0 && Math.abs(angError) < Math.toRadians(2.0)){
                        driveStage = 4;
                    }
                }
                if(driveStage == 4){
                    err = new Vector(0,0);
                    angle = r.vision.getImageAngle(level, position);
                }
            } else {
                //drive to shelf gather
                double y = 0;
                double x = 0;
                if(target.getY() - r.sensors.odo.botLocation.getY() > 0.0){
                    y = -AutonPos.GATHER_Y_DIFF + 7;
                    x = AutonPos.GATHER_X_DIFF_R + r.sensors.gatherDistOffset;
                } else {
                    y = AutonPos.GATHER_Y_DIFF;
                    x = AutonPos.GATHER_X_DIFF_L + r.sensors.gatherDistOffset;
                }

                if(driveStage == 0){
                    double currX = r.sensors.odo.botLocation.getX();
                    stage1ShelfXTarget = Math.min(currX + 24, target.getX() + x - 36);
                    driveStage = 1;
                }
                if(driveStage == 1){
                    pwrMultiplier = 0.3;
                    pwrMax = PWR_MAX_CUBE;
                    //Move it to the correct y position and rotate
                    Vector yAlign = Vector.fromXY(stage1ShelfXTarget, target.getY() + y);
                    err = Vector.subVectors(yAlign, r.sensors.odo.botLocation);
                    angle = 0;
                    
                    if(err.r < 3){
                        driveStage = 2;
                    }
                }
                if(driveStage == 2){
                    pwrMultiplier = 0.3;
                    pwrMax = PWR_MAX_CUBE;
                    //Move it to the correct y position and rotate
                    Vector yAlign = Vector.fromXY(stage1ShelfXTarget, target.getY() + y);
                    err = Vector.subVectors(yAlign, r.sensors.odo.botLocation);
                    angle = 0;

                    if(r.arm.setPoint != null && Math.abs(r.arm.setPoint.theta - Math.toRadians(r.arm.angleMotor.getPosition())) < Math.toRadians(15)){
                        driveStage = 3;
                        mpStartLoc = new Vector(r.sensors.odo.botLocation);
                        mpStartVel = 0;
                        mpStartTime = Timer.getFPGATimestamp();
                        setMotionProfiling(true);
                    }
                }
                if(driveStage == 3){

                    pwrMultiplier = 0.35;
                    pwrMax = PWR_MAX_GATHER;
                    //drive in
                    Vector offsetTarget = Vector.fromXY(target.getX() + x, target.getY() + y);
                    err = Vector.subVectors(offsetTarget, r.sensors.odo.botLocation);
                    angle = 0;

                    if(motionProfiling == true){
                        //Motion Profile
                        mpPwr = getMPPwr(mpStartLoc, mpStartVel, mpStartTime, offsetTarget);
                        if(Timer.getFPGATimestamp() - mpStartTime > mpCompletionTime){
                            driveStage = 4;
                            setMotionProfiling(false);
                        }
                    } else {
                        if(err.r < 2){
                            driveStage = 4;
                        }
                    }
                }
                if(driveStage == 4){
                    err = new Vector(0,0);
                    angle = 0;
                }
            }

            if(debug) System.out.println("Stage" + driveStage + " Error: " + err.toString());
            SmartDashboard.putString("ImageVector", err.toString());

            //account for field oriented
            power = new Vector(err);
            power.theta -= r.sensors.odo.botAngle;

            /*double iPwr = 0;
            if(power.r < 4){
                iPwr += (power.r * r.sensors.dt);
            }*/

            power.r = ((power.r / 12.0) * pwrMultiplier) /*+ iPwr*/;/*power per foot of error*/
            if(power.r > pwrMax) power.r = pwrMax;

            if((Math.abs(r.inputs.getJoystickX()) > 0.25
            || Math.abs(r.inputs.getJoystickY()) > 0.25) 
            && !motionProfiling){
                power = getJoystickPower();
            }
        }

        double zCmd = getJoystickAngle();
        if(Math.abs(zCmd) > 0.1 && !motionProfiling){//manual override
            r.driveTrain.driveSwerve(power, zCmd);
        } else if(motionProfiling){//motion profiled steps
            r.driveTrain.swerveMP(mpPwr, angle);
        } else {//driving to a position
            r.driveTrain.driveSwerveAngle(power, angle);
        }
    }

    private void setMotionProfiling(boolean mode){
        motionProfiling = mode;
        System.out.println("setting mp to " + mode);
    }

    private Vector getJoystickPower(){
        Vector power = Vector.fromXY(-r.inputs.getJoystickY(), -r.inputs.getJoystickX());
        if(r.inputs.getFieldOrient()){
            power.theta -= r.sensors.odo.botAngle;
        }

        //Square inputs for smoother driving
        power.r = power.r * power.r;

        //Field mode v. pit mode
        if(r.inputs.scoringSlowMode){
            power.r *= r.driveTrain.cals.scoringStrafePwr;
        } else if(r.inputs.getFieldMode()){
            power.r *= r.driveTrain.cals.fieldModePwr;
        } else {
            power.r *= r.driveTrain.cals.pitModePwr;
        }

        return power;
    }

    private double getJoystickAngle(){
        double z = r.inputs.getJoystickZR();

        //Square inputs for smoother driving
        z = z * z * Math.signum(z);

        //Field mode v. pit mode
        if(r.inputs.scoringSlowMode){
            z *= r.driveTrain.cals.scoringRotPwr;
        } else if(r.inputs.getFieldMode()){
            z *= r.driveTrain.cals.fieldModePwr;
        } else {
            z *= r.driveTrain.cals.pitModePwr;
        }

        return z;
    }

    @Override
    public boolean isFinished(){
        return driveStage > 3 || (r.inputs.getLeftTrigger() && scoreMode);
    }

    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
        r.vision.allOff();
    }


    Vector getMPPwr(Vector startPos, double startVel, double startTime, Vector endPos){
        
        Vector totalDistance = Vector.subVectors(endPos, startPos);
        double distThreshold = (mpCals.maxVel * mpCals.maxVel) / mpCals.maxAccel;

        double accelDist;
        double decelDist;
        double maxVelDist;
        double accelTime;
        double maxVelTime;
        double decelTime;
        if (totalDistance.r >= distThreshold){
            //3 step (Accel - constV - Decel)
            accelDist = distThreshold / 2;
            decelDist = distThreshold / 2;
            maxVelDist = totalDistance.r - distThreshold;
            accelTime =  Math.sqrt(distThreshold / mpCals.maxAccel);
            //System.out.println("" + accelTime);
            maxVelTime = accelTime + maxVelDist / mpCals.maxVel;
            decelTime = maxVelTime + accelTime;
        } else{
            //2 step (Accel - Decel)
            maxVelTime = 0;
            maxVelDist = 0;
            accelDist = totalDistance.r / 2;
            decelDist = totalDistance.r / 2;
            accelTime =  Math.sqrt(totalDistance.r / mpCals.maxAccel);
            decelTime = 2 * accelTime;
        }

        mpCompletionTime = decelTime;

        

        double t = Timer.getFPGATimestamp() - mpStartTime;
        Vector targetPos = new Vector(0,totalDistance.theta);
        Vector targetVel = new Vector(0,totalDistance.theta);
        double targetAccel = 0;
        if (t < accelTime){ 
            //stage 1
            targetAccel = mpCals.maxAccel;
            targetPos.r = 0.5 * targetAccel * t * t;
            targetVel.r = targetAccel * t;
        } else if (t < maxVelTime) { 
            //stage 2
            targetAccel = 0;
            targetVel.r = mpCals.maxVel;
            targetPos.r = ((t - accelTime) * targetVel.r) + accelDist;
        } else if (t < decelTime) { 
            //stage 3
            double t3 = t - decelTime;
            targetAccel = -mpCals.maxAccel;
            targetPos.r = 0.5 * targetAccel * t3 * t3 + totalDistance.r;
            targetVel.r = t3 * targetAccel;
        } else { 
            //end
            targetPos.r = totalDistance.r;
            targetVel.r = 0;
            targetAccel = 0;
        }

        //PID
        Vector distVec = new Vector(mpStartLoc).sub(r.sensors.odo.botLocation).add(targetPos);
        double errorMag = distVec.r;
        distVec.r *= mpCals.kP_MP;
        Vector totalVel = new Vector(targetVel).add(distVec);

        //output
        totalVel.r *= mpCals.kV;
        Vector power = new Vector(mpCals.kA * targetAccel, targetVel.theta).add(totalVel);
        power.r += mpCals.kS;

        //voltage compensation
        double volts = r.lights.pdh.getVoltage();
        if(volts < 5 || volts > 15){
            volts = 12;
        }
        power.r *= 12.0 / volts;

        if(debug) System.out.format("t:%.2f, p:%.2f, err:%.0f, x:%.0f, v:%.0f, a:%.0f, t1:%.1f, t2:%.1f, t3:%.1f\n", t,power.r,errorMag,targetPos.r,totalVel.r/mpCals.kV,targetAccel,accelTime,maxVelTime,decelTime);

        return power;
    }
}
