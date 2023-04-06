package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class DriveToImage extends CommandBase{
    
    RobotContainer r;

    boolean scoreMode;
    
    public int driveStage;
    public Vector err;

    boolean debug = true;

    double maxFilterDist = 1.0;//inches
    double maxSingleFrameOffset = 3;//inches
    double filterDivisor = 4.0;

    public DriveToImage(RobotContainer r, boolean scoreMode){
        this.r = r;
        this.scoreMode = scoreMode;
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
    }

    public Vector target;

    double pwrMultiplier;
    double pwrMax;
    final double PWR_MAX_CUBE = 0.3;
    final double PWR_MAX_CONE = 0.2;
    final double PWR_MAX_SHELF_GATHER = 0.45;
    final double PWR_MAX_GROUND_GATHER = 0.3;

    public double angle;

    public int level;
    public int position;

    @Override
    public void execute(){
        position = (r.inputs.selectedZone.ordinal() - 1) * 3 + r.inputs.selectedPosition.ordinal();
        
        if(target == null){
            Vector v = r.vision.getImageVector(level, position, scoreMode);
            if(v != null) {
                target = new Vector(v);
                if(debug) System.out.print("Raw: " + v.toString());
            }
        } else {
            Vector newImage = r.vision.getImageVector(level, position, scoreMode);
            if(newImage != null){
                if(debug) System.out.print("Raw: " + newImage.toString());
                Vector delta = Vector.subVectors(newImage, target);
                if(delta.r > maxFilterDist){
                    delta.r /= filterDivisor;
                    if(delta.r > maxSingleFrameOffset){
                        delta.r = maxSingleFrameOffset;
                    }
                    target.add(delta);
                }
            }
        }



        Vector power;
        if(target == null){
            if(scoreMode){
                angle = Math.PI;
            } else {
                angle = 0;
            }
            power = getJoystickPower();

        } else {
            if(debug) System.out.println(" Target: " + target.toString());
            if(driveStage == 0) driveStage = 1;

            //different logic based on if you're gathering or scoring
            if(scoreMode){

                //no else cases, so when we move to next stage we
                //immediately take the new drive action of that stage
                double coneMidOffset = 0;
                if(!r.inputs.isCube()) coneMidOffset = 10;
                if(driveStage == 1){
                    pwrMultiplier = 0.4;
                    pwrMax = PWR_MAX_CUBE;
                    //Move it to a mid-substation x position first
                    Vector xOffset = Vector.fromXY(target.getX() + AutonPos.tagToMidX + coneMidOffset, r.sensors.odo.botLocation.getY());
                    err = Vector.subVectors(xOffset, r.sensors.odo.botLocation);
                    angle = Math.PI;
                    if(Math.abs(err.getX()) < 4.0){
                        driveStage = 2;
                    }
                } 
                if(driveStage == 2){
                    pwrMultiplier = 0.35;
                    pwrMax = PWR_MAX_CUBE;
                    //Move it to the correct y position next
                    Vector yAlign = Vector.fromXY(target.getX() + AutonPos.tagToMidX + coneMidOffset, target.getY());
                    err = Vector.subVectors(yAlign, r.sensors.odo.botLocation);
                    angle = r.vision.getImageAngle(level, position);
                    
                    if(err.r < 1.0){
                        driveStage = 3;
                    }
                }
                if(driveStage == 3) {
                    pwrMultiplier = 0.3;
                    
                    //Final drive in
                    err = Vector.subVectors(target,r.sensors.odo.botLocation);
                    angle = r.vision.getImageAngle(level, position);
                    if(err.r < 40 && !r.inputs.isCube()){
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

                double y = 0;
                if(target.getY() - r.sensors.odo.botLocation.getY() > 0.0){
                    y = -AutonPos.GATHER_Y_DIFF;
                } else {
                    y = AutonPos.GATHER_Y_DIFF;
                }
                
                if(driveStage == 1){
                    pwrMultiplier = 0.3;
                    pwrMax = PWR_MAX_CUBE;
                    //Move it to the correct y position and rotate
                    Vector yAlign = Vector.fromXY(r.sensors.odo.botLocation.getX(), target.getY() + y);
                    err = Vector.subVectors(yAlign, r.sensors.odo.botLocation);
                    angle = 0;
                    
                    if(err.r < 5){
                        driveStage = 2;
                    }
                }
                if(driveStage == 2){
                    pwrMultiplier = 0.35;
                    pwrMax = PWR_MAX_SHELF_GATHER;
                    //drive in
                    Vector offsetTarget = Vector.fromXY(target.getX(), target.getY() + y);
                    err = Vector.subVectors(offsetTarget, r.sensors.odo.botLocation);
                    angle = 0;

                    if(err.r < 2){
                        driveStage = 4;
                    }
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

            if(Math.abs(r.inputs.getJoystickX()) > 0.1
            || Math.abs(r.inputs.getJoystickY()) > 0.1){
                power = getJoystickPower();
            }
        }

        double zCmd = getJoystickAngle();
        if(Math.abs(zCmd) > 0.1){
            r.driveTrain.driveSwerve(power, zCmd);
        } else {
            r.driveTrain.driveSwerveAngle(power, angle);
        }
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
        return driveStage > 3 || r.inputs.getLeftTrigger();
    }

    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
    }

}
