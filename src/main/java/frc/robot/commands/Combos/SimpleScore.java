package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.util.Vector;

public class SimpleScore extends CommandBase{
    
    public static Command SimpleHiScore(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotAngle(Math.toRadians(15))));

        //move arm to position without using stendo
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiAngle));
        //move angle and stendo to hi hold
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiHold).raceWith(new WaitCommand(3)));
        //move the arm to hi release position
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiRelease));
        //driver backwards and spin gripper backwards
        sg.addCommands((new DriveForTime(r, Vector.fromDeg(0.2, -15), 0)).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(-.3))));
        return sg;
    }
}


