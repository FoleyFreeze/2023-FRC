package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.util.Vector;

public class DriveToImage extends CommandBase{
    
    RobotContainer r;

    boolean scoreMode;
    
    public int driveStage;
    public Vector err;

    double maxFilterDist = 1.0;//inches
    double maxSingleFrameOffset = 3;//inches
    double filterDivisor = 4.0;

    public DriveToImage(RobotContainer r, boolean scoreMode){
        this.r = r;
        this.scoreMode = scoreMode;
    }

    @Override
    public void initialize(){
        target = null;
        pwrMultiplier = 0.1;
        pwrMax = 0.3;

        driveStage = 1;
        err = new Vector(0,0);

        level = r.inputs.selectedLevel.ordinal();
        position = (r.inputs.selectedZone.ordinal() - 1) * 3 + r.inputs.selectedPosition.ordinal();
    }

    Vector target;

    double pwrMultiplier;
    double pwrMax;

    public double angle;

    public int level;
    public int position;

    @Override
    public void execute(){
        
        if(target == null){
            Vector v = r.vision.getImageVector(level, position, scoreMode);
            if(v != null) {
                target = new Vector(v);
                System.out.print("Raw: " + v.toString());
            }
        } else {
            Vector newImage = r.vision.getImageVector(level, position, scoreMode);
            if(newImage != null){
                System.out.print("Raw: " + newImage.toString());
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

        if(target == null){
            if(scoreMode){
                angle = Math.PI;
            } else {
                angle = 0;
            }
            r.driveTrain.driveSwerveAngle(Vector.fromXY(0, 0), angle);
        } else {
            System.out.println(" Target: " + target.toString());

            //different logic based on if you're gathering or scoring
            if(scoreMode){
                //no else cases, so when we move to next stage we
                //immediately take the new drive action of that stage
                double coneMidOffset = 0;
                if(!r.inputs.isCube()) coneMidOffset = 20;
                if(driveStage == 1){
                    pwrMultiplier = 0.4;
                    pwrMax = 0.3;
                    //Move it to a mid-substation x position first
                    Vector xOffset = Vector.fromXY(target.getX() + AutonPos.tagToMidX + coneMidOffset, r.sensors.odo.botLocation.getY());
                    err = Vector.subVectors(xOffset, r.sensors.odo.botLocation);
                    angle = Math.PI;
                    if(err.r < 1.0){
                        driveStage = 2;
                    }
                } 
                if(driveStage == 2){
                    pwrMultiplier = 0.35;
                    pwrMax = 0.3;
                    //Move it to the correct y position next
                    Vector yAlign = Vector.fromXY(target.getX() + AutonPos.tagToMidX + coneMidOffset, target.getY());
                    err = Vector.subVectors(yAlign, r.sensors.odo.botLocation);
                    angle = Math.PI;
                    
                    if(err.r < 1.0){
                        driveStage = 3;
                    }
                }
                if(driveStage == 3) {
                    pwrMultiplier = 0.2;
                    pwrMax = 0.3;
                    //Final drive in
                    err = Vector.subVectors(target,r.sensors.odo.botLocation);
                    angle = r.vision.getImageAngle(level, position);
                    if(err.r < 3.0){
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
                    y = -AutonPos.GATHER_X_DIFF;
                } else {
                    y = AutonPos.GATHER_X_DIFF;
                }
                
                if(driveStage == 1){
                    pwrMultiplier = 0.3;
                    pwrMax = 0.3;
                    //Move it to the correct y position and rotate
                    Vector yAlign = Vector.fromXY(r.sensors.odo.botLocation.getX(), target.getY() + y);
                    err = Vector.subVectors(yAlign, r.sensors.odo.botLocation);
                    angle = 0;
                    
                    if(err.r < 1){
                        driveStage = 2;
                    }
                }
                if(driveStage == 2){
                    pwrMultiplier = 0.15;
                    pwrMax = 0.15;
                    //drive in
                    Vector offsetTarget = Vector.fromXY(target.getX(), target.getY() + y);
                    err = Vector.subVectors(offsetTarget, r.sensors.odo.botLocation);
                    angle = 0;

                    if(err.r < 1){
                        driveStage = 4;
                    }
                }
            }

            System.out.println("Stage" + driveStage + " Error: " + err.toString());
            SmartDashboard.putString("ImageVector", err.toString());

            //account for field oriented
            Vector power = new Vector(err);
            power.theta -= r.sensors.odo.botAngle;

            double iPwr = 0;
            if(power.r < 4){
                iPwr += (power.r * r.sensors.dt);
            }

            power.r = ((power.r / 12.0) * pwrMultiplier) + iPwr;/*power per foot of error*/
            if(power.r > pwrMax) power.r = pwrMax;

            if(Math.abs(r.inputs.getJoystickX()) > 0.1
            || Math.abs(r.inputs.getJoystickY()) > 0.1){
                power = Vector.fromXY(-r.inputs.getJoystickY(), -r.inputs.getJoystickX());
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
            }
            r.driveTrain.driveSwerveAngle(power, angle);
        }
    }

    @Override
    public boolean isFinished(){
        return driveStage > 3;
    }

    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
    }

}
