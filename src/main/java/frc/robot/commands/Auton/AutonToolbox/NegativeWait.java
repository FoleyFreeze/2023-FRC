package frc.robot.commands.Auton.AutonToolbox;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Auton.AdvancedMovement.DriveMotionProfile;

public class NegativeWait extends CommandBase{

    double seconds;
    DriveMotionProfile dmp;

    public NegativeWait(double seconds, DriveMotionProfile dmp){
        this.seconds = seconds;
        this.dmp = dmp;
    }

    @Override
    public boolean isFinished(){
        return Timer.getFPGATimestamp() + seconds > dmp.getTime();
    }
}
