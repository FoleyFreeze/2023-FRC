package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Inputs.Inputs.Level;
import frc.robot.subsystems.Inputs.Inputs.ManScoreMode;
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

    double iAccum = 0;

    @Override
    public void execute(){
        //x and y are flipped between because of the way our axes work; x is forward (0 degrees)
        Vector xy = Vector.fromXY(-r.inputs.getJoystickY(), -r.inputs.getJoystickX());
        if(r.inputs.getFieldOrient()){
            xy.theta -= r.sensors.odo.botAngle;
        }

        double z = r.inputs.getJoystickZR();

        //Square inputs for smoother driving
        xy.r = xy.r * xy.r;
        z = z * z * Math.signum(z);

        //auto align
        if(true /*r.inputs.getFieldAlign()*/){
            if (z != 0){
                lastRotateTime = Timer.getFPGATimestamp();
                iAccum = 0;
            } else if(Timer.getFPGATimestamp() - lastRotateTime > r.driveTrain.cals.autoAlignWaitTime) {
                //select setpoint
                double setpoint = 999;
                if(r.inputs.autoGather.getAsBoolean() && r.inputs.isShelf()){
                    //target shelf angle
                    setpoint = Math.toRadians(0);
                } else if(r.inputs.scoreMode == ManScoreMode.SCORE && r.inputs.selectedLevel == Level.TOP && !r.inputs.isCube()){
                    //target score angle for lvl3 cones
                    if(r.inputs.fieldAlignRight.getAsBoolean()){
                        setpoint = Math.toRadians(-164.25);
                    } else {
                        setpoint = Math.toRadians(-195);
                    }
                }

                //if we have an in bounds setpoint
                if(Math.abs(setpoint) <= Math.PI*2){
                    //error is between -360 -> 360
                    double error = (setpoint - r.sensors.odo.botAngle) % (2*Math.PI);
                    
                    if(Math.abs(error) > Math.PI){
                        if(error > 0) error -= 2*Math.PI;
                        else error += 2*Math.PI;
                    }

                    if(error > Math.toRadians(8)){
                        //keep i low to prevent oscillation
                        iAccum = 0;
                    } else {
                        iAccum += error * r.sensors.dt;
                    }
                    z = error * r.driveTrain.cals.autoAlignKp + iAccum * r.driveTrain.cals.autoAlignKi;
                    z = Util.bound(z, -r.driveTrain.cals.autoAlignMaxPower, r.driveTrain.cals.autoAlignMaxPower);
                    SmartDashboard.putNumber("Z",z);
                } else {
                    //nothing to auto align to
                    z=0;
                    iAccum = 0;
                }
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

        //Auto-align to substation logic
        /*
        if(r.inputs.autoGather.getAsBoolean() && r.inputs.isShelf() && Math.abs(r.inputs.getJoystickZR()) < 0.2){
            double error = (0 - r.sensors.odo.botAngle) % (2 * Math.PI);
                
            if(Math.abs(error) > Math.PI){
                if(error > 0) error -= 2 * Math.PI;
                else error += 2 * Math.PI;
            }

            z = Util.bound(error * 0.4, -0.4, 0.4);
        }
        */

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
