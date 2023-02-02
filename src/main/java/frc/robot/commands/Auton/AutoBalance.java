package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.BasicMovement.DistanceDrive;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.util.Vector;

public class AutoBalance {

    public static Command getAutoBalanceCommand(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        //driving sideways(roll) for 3 seconds or until we make it most of the way up the thing
        sg.addCommands(new DriveForTime(r, Vector.fromXY(0, .2), 3).until(() -> r.sensors.navX.getRoll() > 33));
        //now wait until the charge station starts falling down
        sg.addCommands(new DriveForTime(r, Vector.fromXY(0, .2), 1).until(() -> r.sensors.navX.getRoll() < 30));
        //stop until we are at the wanted pitch/roll
        sg.addCommands(new InstantCommand(r.driveTrain::parkMode, r.driveTrain).until(() -> r.sensors.navX.getRoll() < 2));
        //move back a little bit
        sg.addCommands(new DriveForTime(r, Vector.fromXY(0, .2), .5));
        //park
        sg.addCommands(new InstantCommand(r.driveTrain::parkMode, r.driveTrain));
        //wait
        sg.addCommands(new WaitCommand(1.54));
        //adjust
        sg.addCommands(new ConditionalCommand(new WaitCommand(5), 
                                            new ConditionalCommand(new DriveForTime(r, Vector.fromXY(0, -.2), .3), 
                                                                   new DriveForTime(r, Vector.fromXY(0, .2), .3), 
                                                                   () -> r.sensors.navX.getRoll() < -10), 
                                            () -> r.sensors.navX.getRoll() > -10 && r.sensors.navX.getRoll() < 10));

        return sg;
    }




    
    public static Command getAutoBalanceDistCommand(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        //driving sideways(roll) for 3 seconds or until we make it most of the way up the thing
        sg.addCommands(new DistanceDrive(r, Vector.fromXY(0, 60)).until(() -> r.sensors.navX.getRoll() > 33));
        //now wait until the charge station starts falling down
        sg.addCommands(new DistanceDrive(r, Vector.fromXY(0, 18)).until(() -> r.sensors.navX.getRoll() < 30));
        //stop until we are at the wanted pitch/roll
        sg.addCommands(new InstantCommand(r.driveTrain::parkMode, r.driveTrain).until(() -> r.sensors.navX.getRoll() < 2));
        //move back a little bit
        sg.addCommands(new DistanceDrive(r, Vector.fromXY(0, -3)));
        //park
        sg.addCommands(new InstantCommand(r.driveTrain::parkMode, r.driveTrain));
        //wait
        sg.addCommands(new WaitCommand(1.54));
        //adjust
        sg.addCommands(new ConditionalCommand(new WaitCommand(5), 
                                            new ConditionalCommand(new DistanceDrive(r, Vector.fromXY(0, -3)), 
                                                                   new DistanceDrive(r, Vector.fromXY(0, 3)), 
                                                                   () -> r.sensors.navX.getRoll() < -10), 
                                            () -> r.sensors.navX.getRoll() > -10 && r.sensors.navX.getRoll() < 10));
        
        return sg;
    }

}