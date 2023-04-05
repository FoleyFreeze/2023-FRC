package frc.robot.commands.Drive;

import java.time.LocalDate;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors.VisionData;
import frc.robot.subsystems.Sensors.VisionDataEntry;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class DriveToGamePiece extends CommandBase{
    private RobotContainer r;

    private boolean cubeMode;
    private Vector target;
    private double angle;

    public DriveToGamePiece(RobotContainer r){
        this.r = r;
        addRequirements(r.driveTrain);
    }

    @Override
    public void initialize(){
        cubeMode = r.inputs.isCube();
        if(cubeMode){
            r.vision.setCubeMode();
        } else {
            r.vision.setConeMode();
        }
    }

    @Override 
    public void execute(){
        VisionDataEntry vde = null;
        if(cubeMode){
            vde = r.vision.cubeVisionStack.pop();
        } else {
            vde = r.vision.coneVisionStack.pop();
        }

        if(vde != null){
            //update target
            VisionData vd = vde.listFin.get(0);
            double localAngle = Math.toRadians(vd.pose.getRotation().getY());
            double localDist = vd.pose.getTranslation().getZ();

            angle = localAngle + r.sensors.odo.botAngle;
            Vector target = new Vector(localDist, 0);
        }

        if(target != null){
            //drive to target
            Vector power = new Vector(target);
            if(Angle.normRad(r.sensors.odo.botAngle - angle) < Math.toRadians(5)){
                power.r = target.r * 0.1;
                if(power.r > 0.25) power.r = 0.25;
            } else {
                power.r = 0;
            }
            r.driveTrain.driveSwerveAngle(power, angle);
        } else {
            driveManual();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(new Vector(0,0), 0);
    }

    private void driveManual(){
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
}
