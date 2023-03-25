package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.AutonToolbox.AutonCommand;
import frc.robot.commands.Auton.AutonToolbox.WaitForStage;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.commands.Drive.AutoAlign;
import frc.robot.commands.Drive.DriveToImage;
import frc.robot.commands.Gripper.GatherCommand;
import frc.robot.subsystems.Inputs.Inputs.ManScoreMode;
import frc.robot.util.Vector;

public class CamCommands extends SequentialCommandGroup{

    public static Command AutoDriveToScore(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        sg.addCommands(new InstantCommand(() -> r.inputs.setMode(ManScoreMode.UP)));
        DriveToImage dti = new DriveToImage(r, true);
        sg.addCommands(dti.alongWith(new WaitForStage(r, 10, dti, true)
                                    .andThen(new ArmMove(r, r.inputs.armScorePos))));
        
        sg.addCommands(new ConditionalCommand(scoreOnlyCube(r), 
                                              scoreOnlyCone(r), 
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

    public static Command scoreOnlyCone(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        sg.addCommands(new ConditionalCommand(AutoAlign.autoFieldLeftAlign(r), AutoAlign.autoFieldRightAlign(r), () -> determineLeftAlignment(r.inputs.selectedZone.ordinal(), r.inputs.selectedPosition.ordinal())));
        sg.addCommands(new InstantCommand(() -> r.inputs.setMode(ManScoreMode.SCORE)));
        //move the arm to release position
        sg.addCommands(new ArmMove(r, r.inputs.armScorePos));
        //driver backwards
        sg.addCommands((new DriveForTime(r, Vector.fromXY(0.25, 0), 0.9)));
        //lower arm
        sg.addCommands(new ArmGoHome(r).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));
    
        return sg;
    }

    static boolean determineLeftAlignment(int zone, int pos){
        if(zone == 1 || zone == 2){
            if(pos == 1){
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    public static Command scoreOnlyCube(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        sg.addCommands(new WaitCommand(0.27));
        //eject it
        sg.addCommands(new RunCommand(() -> r.gripper.setIntakePower(r.gripper.cals.cubeScorePower), r.gripper).raceWith(new WaitCommand(0.2)));
        sg.addCommands(new InstantCommand(() -> r.inputs.setMode(ManScoreMode.SCORE)));
        //drive backwards
        sg.addCommands(new DriveForTime(r, Vector.fromXY(0.25, 0), 0.4));
        //lower arm
        sg.addCommands(new ArmGoHome(r).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));

        return sg;
    }
}
