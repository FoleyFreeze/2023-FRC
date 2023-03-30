package frc.robot.commands.Auton.AutonToolbox;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
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
        sg.addCommands(new DriveForTime(r, Vector.fromXY(.25*s, 0), 0.5));//drive a bit before checking angle again
        //now wait until the charge station starts falling down
        sg.addCommands(new DriveForTime(r, Vector.fromXY((.17+0.03)*s, 0), 10.0).until(() -> r.sensors.getAbsPitchRoll() < 11.5));
        //stop until we are at the wanted pitch/roll
        sg.addCommands(new InstantCommand(() -> r.driveTrain.setParkMode(true), r.driveTrain));
        sg.addCommands(new InstantCommand(() -> r.driveTrain.driveSwerve(new Vector(0,0),0)));//to make the park command take
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

    //Goes over and resets x-pos
    public static Command getDriveOverStation(RobotContainer r, boolean toSubstation){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        double direction;
        double endX;
        if(toSubstation){
            direction = 180;
            endX = 117.0 - 13.0;
        } else {
            direction = 0;
            endX = 193.25 + 13.0;
        }

        sg.addCommands(new DriveForTime(r, Vector.fromDeg(0.2, direction), 4.0).until(() -> r.sensors.getAbsPitchRoll() > 15.0));
        sg.addCommands(new DriveForTime(r, Vector.fromDeg(0.2, direction), 3.0).until(() -> r.sensors.getAbsPitchRoll() < 5.0));
        sg.addCommands(new DriveForTime(r, Vector.fromDeg(0.2, direction), 2.0).until(() -> r.sensors.getAbsPitchRoll() > 8.0));
        sg.addCommands(new DriveForTime(r, Vector.fromDeg(0.2, direction), 1.0).until(() -> r.sensors.getAbsPitchRoll() < 3.0));

        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotLocation(Vector.fromXY(endX, r.sensors.odo.botLocation.getY()))));

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