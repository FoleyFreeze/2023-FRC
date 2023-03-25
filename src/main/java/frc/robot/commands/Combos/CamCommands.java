package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.AutonToolbox.AutonCommand;
import frc.robot.commands.Auton.AutonToolbox.WaitForStage;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.commands.Drive.DriveToImage;
import frc.robot.commands.Gripper.GatherCommand;
import frc.robot.util.Vector;

public class CamCommands extends SequentialCommandGroup{

    public static Command AutoDriveToScore(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        DriveToImage dti = new DriveToImage(r, true);
        sg.addCommands(dti.alongWith(new WaitForStage(r, 25, dti, true)
                                    .andThen(new ArmMove(r, r.inputs.armScorePos))));
        
        sg.addCommands(new ConditionalCommand(AutonCommand.scoreOnlyCube(r), 
                                              AutonCommand.scoreOnlyCone(r), 
                                              r.inputs::isCube));

        return sg.handleInterrupt(() -> r.gripper.setIntakePower(0.07));
    }

    public static Command AutoDriveToGather(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        DriveToImage dti = new DriveToImage(r, false);
        sg.addCommands(dti.raceWith(new WaitForStage(r, 0/*This # is obsolete here*/, dti, false)
                                    .andThen(GatherCommand.gatherCommand(r))));

        sg.addCommands(new DriveForTime(r, Vector.fromXY(-0.25, 0.0), 0.9)
                       .andThen(new ArmGoHome(r)));

        return sg.handleInterrupt(() -> r.gripper.setIntakePower(0.07));
    }
}
