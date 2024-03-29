package frc.robot.commands.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.subsystems.Inputs.Inputs;

public class GatherCommand {
    
    //set the intake power to .07 (7%) then sets intake power to 0
    public static Command holdIntake(RobotContainer r){
        return new RunCommand(() -> r.gripper.setIntakePower(0.07), r.gripper)
                            .finallyDo((a) -> r.gripper.setIntakePower(0));
    }
    //run the gripper backwards .5 power (50%) for .3 secs then sets intake power to 0
    public static Command shootIntake(RobotContainer r, boolean inward){
        double intakePwr = -0.5;
        if(inward){
            intakePwr = - intakePwr;
        }
        return new RunCommand(() -> r.gripper.setIntakePower(r.gripper.cals.cubeScorePower), r.gripper)
                            .raceWith(new WaitCommand(0.6))
                            .andThen(new InstantCommand(() -> r.gripper.setIntakePower(r.gripper.cals.pieceHoldPower), r.gripper));
    }

    public static Command gatherCommand(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        sg.addRequirements(r.gripper);
        sg.addRequirements(r.arm);

        //get inputted position
        sg.addCommands(new ArmMove(r, r.inputs::getGatherPosition));

        //runs intake command
        sg.addCommands(new IntakeCommand(r));

        return sg;
    }


}





























        /* 
        if (r.inputs.isCube()) {
            return resg.addCommands(new InstantCommand(r.gripper::setIntakePower(0.5), r.gripper)
                                .until(() -> r.gripper.getCurrent > r.gripper.cals.coneStallCurrent));
        } else{
            return resg.addCommands(new InstantCommand(r.gripper::setIntakePower(0.5), r.gripper)
                                .until(() -> r.gripper.getCurrent > r.gripper.cals.cubeStallCurrent));
        }
         */ 
    

