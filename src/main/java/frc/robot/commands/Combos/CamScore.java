package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.AutonToolbox.AutonCommand;
import frc.robot.commands.Auton.AutonToolbox.WaitForStage;
import frc.robot.commands.Drive.DriveToImage;

public class CamScore extends SequentialCommandGroup{

    public static Command AutoDriveToScore(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        DriveToImage dti = new DriveToImage(r);
        sg.addCommands(dti.alongWith(new WaitForStage(25, dti).andThen(new ArmMove(r, r.inputs.armScorePos))));
        sg.addCommands(new ConditionalCommand(AutonCommand.scoreOnlyCube(r), AutonCommand.scoreOnlyCone(r), r.inputs::isCube));

        return sg.handleInterrupt(() -> r.gripper.setIntakePower(0));
    }
}
