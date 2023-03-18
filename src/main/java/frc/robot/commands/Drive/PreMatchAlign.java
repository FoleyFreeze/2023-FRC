package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonToolbox.AutonCommand;

public class PreMatchAlign extends CommandBase {
    
    RobotContainer r;

    public PreMatchAlign(RobotContainer r){
        this.r = r;

        addRequirements(r.driveTrain);
    }

    @Override
    public void execute(){
        double angle = AutonCommand.startAngle;
        r.driveTrain.driveAngleOnly(angle);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
