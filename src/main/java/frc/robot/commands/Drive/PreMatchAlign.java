package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.MPAutons.AutonCommand;

public class PreMatchAlign extends CommandBase {
    
    RobotContainer r;

    public PreMatchAlign(RobotContainer r){
        this.r = r;

        addRequirements(r.driveTrain);
    }

    @Override
    public void execute(){
        double angle = AutonCommand.startAngle;
        //negative of the target heading (like how field oriented works)
        r.driveTrain.driveAngleOnly(-angle);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
