package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.BasicMovement.DistanceDrive;
import frc.robot.util.Vector;

public class SimpleScore extends CommandBase{
    
    public Command SimpleHiScore(RobotContainer r, boolean isCube){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        if(isCube == false){
            //move arm to position without using stendo
            sg.addCommands((new ArmMove(r, r.arm.cals.positionConeHiAngle)));
            //move angle and stendo to hi hold
            sg.addCommands((new ArmMove(r, r.arm.cals.positionConeHiHold)).raceWith(new WaitCommand(3)));
            //move the arm to hi release position
            sg.addCommands((new ArmMove(r, r.arm.cals.positionConeHiRelease)));
            //driver backwards and spin gripper backwards
            sg.addCommands((new DistanceDrive(r, Vector.fromXY(-18, 0))).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(-.3))));
        }
        return sg;
    }
}


