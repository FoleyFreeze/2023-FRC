package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class GatherCommand {
    
    public static Command holdIntake(RobotContainer r){
        return new RunCommand(() -> r.gripper.setIntakePower(0.07), r.gripper)
                            .finallyDo((a) -> r.gripper.setIntakePower(0));
    }

    public static Command shootIntake(RobotContainer r){
        return new RunCommand(() -> r.gripper.setIntakePower(-0.5), r.gripper)
                            .raceWith(new WaitCommand(0.3))
                            .andThen(new InstantCommand(() -> r.gripper.setIntakePower(0), r.gripper));
    }


    public static Command gatherCommand(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();



        return sg;
    }

}
