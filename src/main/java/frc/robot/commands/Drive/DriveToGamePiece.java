package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Sensors.VisionDataEntry;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class DriveToGamePiece extends CommandBase{
    protected RobotContainer r;

    protected boolean cubeMode;
    protected Vector target;

    Vector piecePosition;

    public DriveToGamePiece(RobotContainer r){
        this(r, Vector.fromXY(0, 0));
    }

    public DriveToGamePiece(RobotContainer r, Vector piecePosition){
        this.r = r;
        this.piecePosition = piecePosition;
        addRequirements(r.driveTrain);
    }

    VisionDataEntry vde;

    double filterConst = 0.5;

    protected double maxRotPwr = 0.2;
    protected double rotPwr = 0.25;

    protected double maxDrivePwr = 0.5;
    protected double drivePwr = 0.1;

    boolean debug = true;
    int stage;

    Vector startLocation;
    double totalDist;

    double startTime;

    @Override
    public void initialize(){
        target = null;
        stage = 0;

        if(DriverStation.isAutonomous()){//force cube pickup in auton
            cubeMode = true;
        } else {
            cubeMode = r.inputs.isCube();
        }

        if(cubeMode){
            r.vision.setCubeMode();
        } else {
            r.vision.setConeMode();
        }

        startTime = Timer.getFPGATimestamp();
        startLocation = new Vector(r.sensors.odo.botLocation);
        totalDist = 0;
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
            } else if(stage==0){
                //filter in new target
                newTarget.sub(target);
                if(newTarget.r < 24){
                    newTarget.r *= filterConst;
                    target.add(newTarget);
                    totalDist = Vector.subVectors(target, r.sensors.odo.botLocation).r;
                }
            }
        }

        if(target != null){
            //don't drive in auton until the piece is close enough to its real position
            if(DriverStation.isAutonomous() && Vector.subVectors(piecePosition, target).r > 24) {
                target = null;
                r.driveTrain.driveSwerve(new Vector(0,0), 0);
                return;
            }

            //drive to target

            //gatherer is 27in out at robot angle of 0
            double gatherExt = 13+11;
            Vector gatherLoc = new Vector(gatherExt,r.sensors.odo.botAngle);
            gatherLoc.add(r.sensors.odo.botLocation);

            Vector driveVec = Vector.subVectors(target, gatherLoc);

            //drive to angle
            double angleError = Angle.normRad(driveVec.theta - r.sensors.odo.botAngle);
            double anglePower = angleError * rotPwr;
            if(anglePower > maxRotPwr) anglePower = maxRotPwr;
            else if(anglePower < -maxRotPwr) anglePower = -maxRotPwr;
            //strafe vector to add to rotation so that we rotate around the gatherer
            Vector drivePower = Vector.fromXY(0, -(gatherExt/13.1)*anglePower);

            double dynamicDelay = Math.max(0,(3 - driveVec.r/12.0)*0.3);
            driveVec.add(new Vector(12,r.sensors.odo.botAngle));
            
            if(stage == 1 || Math.abs(angleError) < Math.toRadians(5) && (Timer.getFPGATimestamp() - startTime) > dynamicDelay){
                stage = 1;
                driveVec.r *= drivePwr;
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
        //Finished when gripper sees current
        //or if we have driven 2ft farther than we needed to
        //if(stage > 0) return true;

        double totalCurr = r.gripper.getIntakeCurrent();
        double dist = Vector.subVectors(r.sensors.odo.botLocation, startLocation).r;
        return startTime + 0.75 < Timer.getFPGATimestamp() 
               && totalCurr > 20
               || totalDist != 0 && dist > totalDist + 24;
    }

    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(new Vector(0,0), 0);
        r.vision.allOff();
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
