package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.util.Vector;

public class DriveToImage extends CommandBase{
    
    RobotContainer r;
    
    public int driveStage;
    public Vector err;

    double maxFilterDist = 1.0;//inches
    double maxSingleFrameOffset = 3;//inches
    double filterDivisor = 4.0;

    public DriveToImage(RobotContainer r){
        this.r = r;
        addRequirements(r.driveTrain);
    }

    @Override
    public void initialize(){
        target = null;
        driveStage = 1;
        err = new Vector(0,0);
    }

    Vector target = null;

    @Override
    public void execute(){
        int level = r.inputs.selectedLevel.ordinal();
        int position = (r.inputs.selectedZone.ordinal() - 1) * 3 + r.inputs.selectedPosition.ordinal();
        
        if(target == null){
            Vector v = r.vision.getImageVector(level, position);
            if(v != null) {
                target = new Vector(v);
                System.out.print("Raw: " + v.toString());
            }
        } else {
            Vector newImage = r.vision.getImageVector(level, position);
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
            r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
        } else {
            System.out.println(" Target: " + target.toString());

            double angle = 0;
            //no else cases, so when we move to next stage we
            //immediately take the new drive action of that stage
            if(driveStage == 1){
                //Move it to a mid-substation x position first
                Vector xOffset = Vector.fromXY(target.getX() + AutonPos.tagToMidX, r.sensors.odo.botLocation.getY());
                err = Vector.subVectors(xOffset, r.sensors.odo.botLocation);
                angle = Math.PI;
                if(err.r < 1.0){
                    driveStage = 2;
                }
            } 
            if(driveStage == 2){
                //Move it to the correct y position next
                Vector yAlign = Vector.fromXY(target.getX() + AutonPos.tagToMidX, target.getY());
                err = Vector.subVectors(yAlign, r.sensors.odo.botLocation);
                angle = Math.PI;
                
                if(err.r < 1.0){
                    driveStage = 3;
                }
            }
            if(driveStage == 3) {
                //Final drive in
                err = Vector.subVectors(target,r.sensors.odo.botLocation);
                angle = r.vision.getImageAngle(level, position);
                if(err.r < 1.0){
                    driveStage = 4;
                }
            }
            if(driveStage == 4){
                err = new Vector(0,0);
                angle = r.vision.getImageAngle(level, position);
            }

            System.out.println("Stage" + driveStage + " Error: " + err.toString());
            SmartDashboard.putString("ImageVector", err.toString());

            //account for field oriented
            Vector power = new Vector(err);
            power.theta -= r.sensors.odo.botAngle;

            power.r *= 0.1;
            if(power.r > 0.1) power.r = 0.1;

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
