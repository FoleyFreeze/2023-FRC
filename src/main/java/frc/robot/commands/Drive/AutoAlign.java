package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class AutoAlign {
    public static Command oldFieldLeftAlign(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromDeg(0.1, -75),0), r.driveTrain).raceWith(new WaitCommand(0.2)));
        //                                                                          x   y   rot
        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(.16, 0),0), r.driveTrain).raceWith(new WaitCommand(0.55)));
        //sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0, 0),-0.15), r.driveTrain).raceWith(new WaitCommand(0.3)));

        return sg;
    }

    public static Command oldFieldRightAlign(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromDeg(0.1, 75),0), r.driveTrain).raceWith(new WaitCommand(0.2)));
        //                                                                          x   y   rot
        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(.16, 0),0), r.driveTrain).raceWith(new WaitCommand(0.55)));
        //sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0, 0),0.15), r.driveTrain).raceWith(new WaitCommand(0.3)));

        return sg;
    }

    public static Command autoFieldRightAlign(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        //sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0.2, 0), 0)).raceWith(new WaitCommand(0.2)));
        //sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0, 0.2), 0)).raceWith(new WaitCommand(0.25)));
        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0.05, 0.15), 0)).raceWith(new WaitCommand(0.25)));

        final double pwr = 0.05;
        final double x = 3.34;//1.4;
        final double y = -2.9;//-2.4;
        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY((y/13.1)*pwr, -(x/13.1)*pwr), pwr)).raceWith(new WaitCommand(1.20)));

        sg.addCommands(new InstantCommand(() -> r.driveTrain.setLastRotateTime(Timer.getFPGATimestamp() + r.driveTrain.cals.autoAlignWaitTime)));
        return sg;
    }

    public static Command autoFieldLeftAlign(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        //sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0.2, 0), 0)).raceWith(new WaitCommand(0.2)));
        //sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0, -0.2), 0)).raceWith(new WaitCommand(0.25)));
        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY(0.05, -0.15), 0)).raceWith(new WaitCommand(0.25)));

        final double pwr = -0.05;
        final double x = 3.34;//1.4;
        final double y = 2.9;//2.4;
        sg.addCommands(new RunCommand(() -> r.driveTrain.driveSwerve(Vector.fromXY((y/13.1)*pwr, -(x/13.1)*pwr), pwr)).raceWith(new WaitCommand(1.20)));

        sg.addCommands(new InstantCommand(() -> r.driveTrain.setLastRotateTime(Timer.getFPGATimestamp() + r.driveTrain.cals.autoAlignWaitTime)));
        return sg;
    }
}
