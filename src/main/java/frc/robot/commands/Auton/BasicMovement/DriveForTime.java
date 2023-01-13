package frc.robot.commands.Auton.BasicMovement;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class DriveForTime extends SequentialCommandGroup{
    
    RobotContainer r;

    public DriveForTime(){
        addCommands();
    }
}
