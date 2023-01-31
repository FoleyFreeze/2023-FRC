package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.util.Vector;

public class AutoBalance extends SequentialCommandGroup{

    public AutoBalance(RobotContainer r){
        addRequirements(r.driveTrain);

        //driving sideways(roll) for 3 seconds or until we make it most of the way up the thing
        addCommands((new DriveForTime(r, Vector.fromXY(0, .2), 3)).until(() -> r.sensors.navX.getRoll() > 33));
        //now wait until the charge station starts falling down
        addCommands((new DriveForTime(r, Vector.fromXY(0, .2), 1)).until(() -> r.sensors.navX.getRoll() < 30));
        //stop until we are at the wanted pitch/roll
        addCommands((new InstantCommand(r.driveTrain::parkMode, r.driveTrain)));
        
    }

}