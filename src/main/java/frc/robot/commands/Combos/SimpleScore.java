package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class SimpleScore extends CommandBase{
    
    public static Command SimpleHiScore(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotAngle(Angle.toRad(165))));

        //move arm to position without using stendo
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiAngle));
        //move angle and stendo to hi hold
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiHold).raceWith(new WaitCommand(3)));
        //move the arm to hi release position
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiRelease));
        //driver backwards and spin gripper backwards
        sg.addCommands((new DriveForTime(r, Vector.fromDeg(0.2, 0), 1.25)).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(-.3))));

        sg.addCommands(new DriveForTime(r, Vector.fromDeg(0.2, -90), 1.5).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));

        sg.addCommands((new DriveForTime(r, Vector.fromDeg(0.2, 0), 4.5)).alongWith(new ArmGoHome(r)));
        return sg;
    }
}


