package frc.robot.commands.Auton.AutonToolbox;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    
    public static Command SimpleHiScore(RobotContainer r, int startPos, boolean balance){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        //sg.addCommands(new InstantCommand(() -> r.arm.setArmOffset(-8.0, 33.5)));

        double[] startAng = {165, 165};

        Vector[] driveOut = {Vector.fromDeg(0.15, 0), Vector.fromDeg(0.15, 0)};
        double[] driveOutTime = {1.25, 1.25};

        Vector[] driveToBalance = {Vector.fromDeg(0.15, 90), Vector.fromDeg(0.15, 90)};
        double[] driveToBalanceTime = {1.5, 1.5};

        Vector[] firstDriveOut = {Vector.fromDeg(0.15, -90), Vector.fromDeg(0.15, -90)};
        double[] firstDriveOutTime = {1.5, 1.5};

        Vector[] secondDriveOut = {Vector.fromDeg(0.15, 0), Vector.fromDeg(0.15, 0)};
        double[] secondDriveOutTime = {4.5, 4.5};

        if(DriverStation.getAlliance() == Alliance.Red){
            for(int i = 0; i < startAng.length; i++){
                startAng[i] = -startAng[i];

                driveOut[i].theta = -driveOut[i].theta;

                driveToBalance[i].theta = -driveToBalance[i].theta;
            }
        }

        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotAngle(Angle.toRad(startAng[startPos]))));

        //move arm to position without using stendo
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiAngle));
        //move angle and stendo to hi hold
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiHold).raceWith(new WaitCommand(3)));
        //move the arm to hi release position
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiRelease));
        //driver backwards and spin gripper backwards
        sg.addCommands((new DriveForTime(r, driveOut[startPos], driveOutTime[startPos])).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(-.3))));


        if(balance){
            sg.addCommands(new DriveForTime(r, driveToBalance[startPos], 90, driveToBalanceTime[startPos]).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));

            sg.addCommands(AutoBalance.basicAutoBalance(r).alongWith(new ArmGoHome(r)));
        } else {
            sg.addCommands(new DriveForTime(r, firstDriveOut[startPos], 90, firstDriveOutTime[startPos]).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));

            sg.addCommands((new DriveForTime(r, secondDriveOut[startPos], secondDriveOutTime[startPos])).alongWith(new ArmGoHome(r)));
        }
        return sg;
    }
}


