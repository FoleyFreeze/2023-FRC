package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class AutoAlign {
    public static Command autoScoreAlign(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

       // sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(new Vector(0.4, 0), 0), r.driveTrain));
       //                                                                           x   y   rot
        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(.05, 0),-.1), r.driveTrain));

        return sg;
    }
}
