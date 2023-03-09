package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Angle;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class CmdDrive extends CommandBase{
    
    RobotContainer r;
    double lastRotateTime; 

    public CmdDrive(RobotContainer r){
        this.r = r;
        addRequirements(r.driveTrain);
    }

    @Override
    public void initialize(){
        lastRotateTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        //x and y are flipped between because of the way our axes work; x is forward (0 degrees)
        Vector xy = Vector.fromXY(-r.inputs.getJoystickY(), -r.inputs.getJoystickX());
        if(r.inputs.getFieldOrient()){
            xy.theta -= r.sensors.odo.botAngle;
        }

        double z = r.inputs.getJoystickZR();
        if(r.inputs.getFieldAlign()){
            if (z != 0){
                lastRotateTime = Timer.getFPGATimestamp();
            } else if(Timer.getFPGATimestamp() - lastRotateTime > r.driveTrain.cals.autoAlignWaitTime) {
                //error is between -180 -> 180
                double error = r.sensors.odo.botAngle % Math.PI;
                
                if(Math.abs(error) > Math.PI/2){
                    if(error > 0) error -= Math.PI;
                    else error += Math.PI;
                }

                z = error * r.driveTrain.cals.autoAlignKp;
                if (z > r.driveTrain.cals.autoAlignMaxPower) z = r.driveTrain.cals.autoAlignMaxPower;
                else if (z < -r.driveTrain.cals.autoAlignMaxPower) z = -r.driveTrain.cals.autoAlignMaxPower;
            }
        }

        //Field mode v. pit mode
        if(r.inputs.scoringSlowMode){
            xy.r *= r.driveTrain.cals.scoringStrafePwr;
            z *= r.driveTrain.cals.scoringRotPwr;
        } else if(r.inputs.getFieldMode()){
            xy.r *= r.driveTrain.cals.fieldModePwr;
            z *= r.driveTrain.cals.fieldModePwr;
        } else {
            xy.r *= r.driveTrain.cals.pitModePwr;
            z *= r.driveTrain.cals.pitModePwr;
        }

        //Square inputs for smoother driving
        xy.r = xy.r * xy.r;
        z = z * z * Math.signum(r.inputs.getJoystickZR());

        //Auto-align to substation logic
        if(r.inputs.autoGather.getAsBoolean() && r.inputs.isShelf() && Math.abs(r.inputs.getJoystickZR()) < 0.2){
            double error = (0 - r.sensors.odo.botAngle) % (2 * Math.PI);
                
            if(Math.abs(error) > Math.PI){
                if(error > 0) error -= 2 * Math.PI;
                else error += 2 * Math.PI;
            }

            z = Util.bound(error * 0.4, -0.4, 0.4);
        }
        
        r.driveTrain.driveSwerve(xy, z);
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
