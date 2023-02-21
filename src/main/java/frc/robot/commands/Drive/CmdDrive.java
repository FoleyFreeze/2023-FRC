package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
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
        if(r.inputs.getFieldMode()){
            xy.r *= r.driveTrain.cals.fieldModePwr;
            z *= r.driveTrain.cals.fieldModePwr;
        } else {
            xy.r *= r.driveTrain.cals.pitModePwr;
            z *= r.driveTrain.cals.pitModePwr;
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
