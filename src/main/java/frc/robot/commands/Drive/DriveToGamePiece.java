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

    public DriveToGamePiece(RobotContainer r){
        this.r = r;
        addRequirements(r.driveTrain);
    }

    VisionDataEntry vde;

    double filterConst = 0;
    double maxRotPwr = 0.2;
    double maxDrivePwr = 0.2;
    boolean debug = true;
    int stage;

    @Override
    public void initialize(){
        target = null;
        stage = 0;

        cubeMode = r.inputs.isCube();
        if(cubeMode){
            r.vision.setCubeMode();
        } else {
            r.vision.setConeMode();
        }
    }

    @Override 
    public void execute(){
        Vector newTarget = null;
        if(cubeMode){
            newTarget = r.vision.getCubeVector();
        } else {
            newTarget = r.vision.getConeVector();
        }

        if(newTarget != null){
            //update target
            if(target == null){
                target = newTarget;
            } else {
                //filter in new target
                newTarget.sub(target);
                newTarget.r *= filterConst;
                target.add(newTarget);
            }
        }

        if(target != null){
            //drive to target

            //gatherer is 27in out at robot angle of 0
            Vector gatherLoc = new Vector(27,r.sensors.odo.botAngle);
            gatherLoc.add(r.sensors.odo.botLocation);

            Vector driveVec = Vector.subVectors(target, gatherLoc);

            //drive to angle
            double angleError = Angle.normRad(driveVec.theta - r.sensors.odo.botAngle);
            double anglePower = angleError * 0.15;
            if(anglePower > maxRotPwr) anglePower = maxRotPwr;
            else if(anglePower < -maxRotPwr) anglePower = -maxRotPwr;
            //strafe vector to add to rotation so that we rotate around the gatherer
            Vector drivePower = Vector.fromXY(0, -2.06*anglePower);

            driveVec.add(new Vector(36,r.sensors.odo.botAngle));

            if(stage == 1 || Math.abs(angleError) < Math.toRadians(5)){
                stage = 1;
                driveVec.r *= 0.05;
                if(driveVec.r > maxDrivePwr) driveVec.r = maxDrivePwr;
                driveVec.theta -= r.sensors.odo.botAngle;
                //drivePower.add(driveVec);
                drivePower = driveVec;
                anglePower = 0;
            }

            Vector loc = Vector.subVectors(target, gatherLoc);
            if(debug) System.out.format("RelTgt: %.0f,%.0f, G: %.0f,%.0f, T: %.0f,%.0f, AngE: %.0f, %.0f, %.0f, Pwr: %.2f,%.2f\n",loc.getX(),loc.getY(),gatherLoc.getX(),gatherLoc.getY(),target.getX(),target.getY(),Math.toDegrees(angleError),Math.toDegrees(loc.theta),Math.toDegrees(r.sensors.odo.botAngle),drivePower.getX(),drivePower.getY());

            r.driveTrain.driveSwerve(drivePower, anglePower);
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
