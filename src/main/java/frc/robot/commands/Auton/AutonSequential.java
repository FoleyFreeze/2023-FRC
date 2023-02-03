package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonToolbox.SetStartPos;

public class AutonSequential extends SequentialCommandGroup{

    RobotContainer r;
    AutonCal cals;
    
    public AutonSequential(RobotContainer r, AutonCal cals){
        this.r = r;
        this.cals = cals;

        int stepIdx = 0;

        //Reset to chosen start position
        addCommands(new SetStartPos(r, cals.START_POSITIONS[r.startPosChooser.getSelected()]));
        stepIdx++;

        
        stepIdx++;
    }
}
