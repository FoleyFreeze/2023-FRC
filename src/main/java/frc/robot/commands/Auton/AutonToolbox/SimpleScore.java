package frc.robot.commands.Auton.AutonToolbox;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class SimpleScore extends CommandBase{
    
    public static Command SimpleHiScore(RobotContainer r, int startPos, boolean balance, Alliance team){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        sg.addCommands(new InstantCommand(() -> r.arm.setArmOffset(AutonPos.initArmAngle, AutonPos.initArmStendo)));

        double[] startAng = {165, 165, -165, 165};

        Vector[] driveOut = {Vector.fromDeg(0.15, 0), Vector.fromDeg(0.15, 0), Vector.fromDeg(0.15, 0), Vector.fromDeg(0.15, 0)};
        double[] driveOutTime = {1.5, 1.5, 1.5, 1.5};

        Vector[] driveToBalance = {Vector.fromDeg(0.1, 90), Vector.fromDeg(0.1, -90), Vector.fromDeg(0, 0), Vector.fromDeg(0, 0)};
        double[] driveToBalanceTime = {2.0, 2.0, 0, 0};
        double driveToBalanceAngle = 90;

        Vector[] firstDriveOut = {Vector.fromDeg(0.15, -90), Vector.fromDeg(0.15, 90), Vector.fromDeg(0.2, 0), Vector.fromDeg(0.2, 0)};
        double[] firstDriveOutTime = {1.5, 1.5, 5.5, 5.5};

        Vector[] secondDriveOut = {Vector.fromDeg(0.15, 0), Vector.fromDeg(0.15, 0), Vector.fromDeg(0, 0), Vector.fromDeg(0, 0)};
        double[] secondDriveOutTime = {2.5, 2.5, 0, 0};

        if(team == Alliance.Red){
            for(int i = 0; i < startAng.length; i++){
                startAng[i] = -startAng[i];

                driveOut[i].theta = -driveOut[i].theta;

                driveToBalance[i].theta = -driveToBalance[i].theta;
                driveToBalanceAngle = - driveToBalanceAngle;
            }
        }

        sg.addCommands(new InstantCommand(() -> r.sensors.resetNavXAng(Angle.toRad(startAng[startPos]))));

        //pull up stendo
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiStendo));
        //move arm to position without using stendo
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiAngle));
        //move angle and stendo to hi hold
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiHold));
        //wait
        sg.addCommands(new WaitCommand(0.2));
        //move the arm to hi release position
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiRelease));
        //driver backwards and spin gripper backwards
        sg.addCommands((new DriveForTime(r, driveOut[startPos], driveOutTime[startPos])).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(-.3))));

        if(balance){
            sg.addCommands(new DriveForTime(r, driveToBalance[startPos], driveToBalanceAngle, driveToBalanceTime[startPos]).alongWith(new ArmGoHome(r)).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));
            sg.addCommands(AutoBalance.getAutoBalanceCommand(r));
        } else {
            sg.addCommands(new DriveForTime(r, firstDriveOut[startPos], firstDriveOutTime[startPos]).alongWith(new ArmGoHome(r)).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));
            sg.addCommands(new DriveForTime(r, secondDriveOut[startPos], secondDriveOutTime[startPos]));
        }
        return sg;
    }
}


