package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class AutoAlign {
    public static Command autoFieldLeftAlign(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromDeg(0.1, -75),0), r.driveTrain).raceWith(new WaitCommand(0.2)));
        //                                                                          x   y   rot
        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(.16, 0),0), r.driveTrain).raceWith(new WaitCommand(0.55)));
        //sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0, 0),-0.15), r.driveTrain).raceWith(new WaitCommand(0.3)));

        return sg;
    }

    public static Command autoFieldRightAlign(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromDeg(0.1, 75),0), r.driveTrain).raceWith(new WaitCommand(0.2)));
        //                                                                          x   y   rot
        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(.16, 0),0), r.driveTrain).raceWith(new WaitCommand(0.55)));
        //sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0, 0),0.15), r.driveTrain).raceWith(new WaitCommand(0.3)));

        return sg;
    }
}
