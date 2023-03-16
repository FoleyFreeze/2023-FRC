package frc.robot.commands.Auton.AutonToolbox;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Auton.BasicMovement.DistanceDrive;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class AutoBalance {


    //Based on time
    public static Command getAutoBalanceCommand(RobotContainer r){
        System.out.println("AutoBalancing");
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);
        
        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotAngle(Angle.toRad(90))));

        //driving sideways(roll) for 3 seconds or until we make it most of the way up the [lifty] thing
        sg.addCommands(new DriveForTime(r, Vector.fromXY(.35, 0), 2.0).until(() -> r.sensors.getAbsPitchRoll() > 25));
        //now wait until the charge station starts falling down
        sg.addCommands(new DriveForTime(r, Vector.fromXY(.17, 0), 10.0).until(() -> r.sensors.getAbsPitchRoll() < 11.5));
        //stop until we are at the wanted pitch/roll
        sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(true), r.driveTrain));
        //sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(false), r.driveTrain));
        //move back a little bit
        //sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(false))
        //    .alongWith(new DriveForTime(r, Vector.fromXY(-.2, 0), .4)));
        //park
        //sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(true), r.driveTrain));
        //wait
        //sg.addCommands(new WaitCommand(1.54));
        //adjust
        /*sg.addCommands(new ConditionalCommand(new WaitCommand(5), 
                                            new ConditionalCommand(new DriveForTime(r, Vector.fromXY(0, -.2), .3), 
                                                                   new DriveForTime(r, Vector.fromXY(0, .2), .3), 
                                                                   () -> r.sensors.navX.getRoll() < -10), 
                                            () -> r.sensors.navX.getRoll() > -10 && r.sensors.navX.getRoll() < 10));*/

        return sg;
    }

    public static Command getAutoBalanceCommand(RobotContainer r, boolean flip){
        double s = 1;
        if(flip){
            s = -1;
        }
        
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.driveTrain);
        
        sg.addCommands(new InstantCommand(() -> System.out.println("AutoBalancing")));

        //sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotAngle(Angle.toRad(90))));

        //driving sideways(roll) for 3 seconds or until we make it most of the way up the [lifty] thing
        sg.addCommands(new DriveForTime(r, Vector.fromXY(.35*s, 0), 2.0).until(() -> r.sensors.getAbsPitchRoll() > 25));
        //now wait until the charge station starts falling down
        sg.addCommands(new DriveForTime(r, Vector.fromXY(.17*s, 0), 10.0).until(() -> r.sensors.getAbsPitchRoll() < 11.5));
        //stop until we are at the wanted pitch/roll
        sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(true), r.driveTrain));
        //sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(false), r.driveTrain));
        //move back a little bit
        //sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(false))
        //    .alongWith(new DriveForTime(r, Vector.fromXY(-.2, 0), .4)));
        //park
        //sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(true), r.driveTrain));
        //wait
        //sg.addCommands(new WaitCommand(1.54));
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


    public static Command basicAutoBalance(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        sg.addCommands(new DriveForTime(r, Vector.fromXY(.4, 0), 3.0));

        sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(false)));

        return sg;
    }

}