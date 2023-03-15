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
    double lastAngle = 0;

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
