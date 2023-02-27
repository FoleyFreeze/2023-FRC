package frc.robot.commands.Auton.AutonToolbox;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Auton.BasicMovement.DistanceDrive;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.util.Vector;

public class AutoBalance {


    //Based on time
    public static Command getAutoBalanceCommand(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        sg.addCommands(new ArmGoHome(r).raceWith(new WaitCommand(1.0)));

        //driving sideways(roll) for 3 seconds or until we make it most of the way up the [lifty] thing
        sg.addCommands(new DriveForTime(r, Vector.fromXY(.35, 0), 2.0).until(() -> r.sensors.getAbsPitchRoll() > 33));
        //now wait until the charge station starts falling down
        sg.addCommands(new DriveForTime(r, Vector.fromXY(.35, 0), 1).until(() -> r.sensors.getAbsPitchRoll() < 30));
        //stop until we are at the wanted pitch/roll
        sg.addCommands(new InstantCommand(r.driveTrain::parkMode, r.driveTrain).until(() -> r.sensors.getAbsPitchRoll() < 2));
        //move back a little bit
        sg.addCommands(new DriveForTime(r, Vector.fromXY(.2, 0), .5));
        //park
        sg.addCommands(new InstantCommand(r.driveTrain::parkMode, r.driveTrain));
        //wait
        sg.addCommands(new WaitCommand(1.54));
        //adjust
        /*sg.addCommands(new ConditionalCommand(new WaitCommand(5), 
                                            new ConditionalCommand(new DriveForTime(r, Vector.fromXY(0, -.2), .3), 
                                                                   new DriveForTime(r, Vector.fromXY(0, .2), .3), 
                                                                   () -> r.sensors.navX.getRoll() < -10), 
                                            () -> r.sensors.navX.getRoll() > -10 && r.sensors.navX.getRoll() < 10));*/

        return sg;
    }




    //Based on distance
    public static Command getAutoBalanceDistCommand(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);

        //driving sideways(roll) for 60 inches or until we make it most of the way up the thing
        sg.addCommands(new DistanceDrive(r, Vector.fromXY(60, 0)).until(() -> r.sensors.getAbsPitchRoll() > 33));
        //now wait until the charge station starts falling down
        sg.addCommands(new DistanceDrive(r, Vector.fromXY(18, 0)).until(() -> r.sensors.getAbsPitchRoll() < 30));
        //stop until we are at the wanted pitch/roll
        sg.addCommands(new InstantCommand(r.driveTrain::parkMode, r.driveTrain).until(() -> r.sensors.getAbsPitchRoll() < 2));
        //move back a little bit
        sg.addCommands(new DistanceDrive(r, Vector.fromXY(-3, 0)));
        //park
        sg.addCommands(new InstantCommand(r.driveTrain::parkMode, r.driveTrain));
        //wait
        sg.addCommands(new WaitCommand(1.54));
        //adjust
        /*sg.addCommands(new ConditionalCommand(new WaitCommand(5), 
                                            new ConditionalCommand(new DistanceDrive(r, Vector.fromXY(0, -3)), 
                                                                   new DistanceDrive(r, Vector.fromXY(0, 3)), 
                                                                   () -> r.sensors.navX.getRoll() < -10), 
                                            () -> r.sensors.navX.getRoll() > -10 && r.sensors.navX.getRoll() < 10));*/
        
        return sg;
    }

}