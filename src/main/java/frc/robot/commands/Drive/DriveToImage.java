package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class DriveToImage extends CommandBase{
    
    RobotContainer r;

    public DriveToImage(RobotContainer r){
        this.r = r;
    }

    @Override
    public void initialize(){
        target = null;
    }

    Vector target = null;

    @Override
    public void execute(){
        if(target == null){
            Vector v = r.vision.getImageVector();
            if(v != null) {
                target = v;
                //add offset
                //TODO: move this logic into getImageVector eventually
                //note that this offset is FIELD RELATIVE
                target.add(Vector.fromXY(40,0));
            }
        }

        if(target == null){
            r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
        } else {
            Vector err = Vector.subVectors(target,r.sensors.odo.botLocation);

            SmartDashboard.putString("ImageVector", err.toString());

            //account for field oriented
            err.theta -= r.sensors.odo.botAngle;

            err.r *= 0.1;
            if(err.r > 0.1) err.r = 0.1;

            r.driveTrain.driveSwerveAngle(err, Math.PI);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        r.driveTrain.driveSwerve(Vector.fromXY(0, 0), 0);
    }

}
