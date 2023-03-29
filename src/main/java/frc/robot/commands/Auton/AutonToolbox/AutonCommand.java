package frc.robot.commands.Auton.AutonToolbox;

import java.time.Instant;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AdvancedMovement.AngleMotionProfile;
import frc.robot.commands.Auton.AdvancedMovement.DriveMotionProfile;
import frc.robot.commands.Auton.AutonCal.MPCals;
import frc.robot.commands.Auton.BasicMovement.DistanceDrive;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.commands.Gripper.GatherCommand;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class AutonCommand {

    public static double startAngle = 0;
    public static Command autonCommand(RobotContainer r, Alliance alliance, int selectedAuton, int startPos){
        
        System.out.println("Creating MP Auton Command");
        SequentialCommandGroup sg = new SequentialCommandGroup();

        Vector[] startPositions = {AutonPos.substation.xy,
                                   AutonPos.subMid.xy,
                                   AutonPos.farMid.xy,
                                   AutonPos.far.xy};
        double[] startAng = {AutonPos.substation.value,
                             AutonPos.subMid.value,
                             AutonPos.farMid.value,
                             AutonPos.far.value};

        Vector[] driveOutOne = {AutonPos.driveSub.xy,
                                AutonPos.driveMid.xy,
                                AutonPos.driveMid.xy,
                                AutonPos.driveFar.xy};
        double[] driveOutOneAng = {AutonPos.driveSub.value,
                                   AutonPos.driveMid.value,
                                   AutonPos.driveMid.value,
                                   AutonPos.driveFar.value};

        Vector[] driveOutTwo = {AutonPos.driveOutSub.xy,
                                AutonPos.driveOutSub.xy,
                                AutonPos.driveOutFar.xy,
                                AutonPos.driveOutFar.xy};

        Vector[] driveToPiece = {AutonPos.drivePieceSub.xy,
                                 AutonPos.drivePieceMidSub.xy,
                                 AutonPos.drivePieceMidFar.xy,
                                 AutonPos.drivePieceFar.xy};
        double[] driveToPieceAng = {AutonPos.drivePieceSub.value,
                                    AutonPos.drivePieceMidSub.value,
                                    AutonPos.drivePieceMidFar.value,
                                    AutonPos.drivePieceFar.value};

        Vector[] driveToScore = {AutonPos.driveScoreSub.xy,
                                 AutonPos.driveScoreSub.xy,
                                 AutonPos.driveScoreFar.xy,
                                 AutonPos.driveScoreFar.xy};
        double[] driveToScoreAng = {AutonPos.driveScoreSub.value,
                                    AutonPos.driveScoreSub.value,
                                    AutonPos.driveScoreFar.value,
                                    AutonPos.driveScoreFar.value};

        MPCals[] mpCalsThere = {AutonCal.driveBase,
                                AutonCal.driveBase,
                                AutonCal.bumpThereCals,
                                AutonCal.bumpThereCals};

        MPCals[] mpCalsBack = {AutonCal.driveBase,
                            AutonCal.driveBase,
                            AutonCal.bumpBackCals,
                            AutonCal.bumpBackCals};

        

        Vector driveToBalanceCommunity = new Vector(AutonPos.driveToBalComm.xy);
        Vector driveToBalanceOutside = new Vector(AutonPos.driveToBalOutside.xy);
        double[] driveToBalanceAngle = {AutonPos.driveToBalComm.value}; //doing this to make it a reference

        if(alliance.equals(Alliance.Red)){
            for(int i = 0; i < 4; i++){
                startPositions[i] = new Vector(startPositions[i]).mirrorY();
                startAng[i] = -startAng[i];

                driveOutOne[i] = new Vector(driveOutOne[i]).mirrorY();
                driveOutOneAng[i] = -driveOutOneAng[i];

                driveOutTwo[i] = new Vector(driveOutTwo[i]).mirrorY();

                driveToPiece[i] = new Vector(driveToPiece[i]).mirrorY();
                driveToPieceAng[i] = -driveToPieceAng[i];

                driveToScore[i] = new Vector(driveToScore[i]).mirrorY();
                driveToScoreAng[i] = -driveToScoreAng[i];
            }
            driveToBalanceCommunity = new Vector(driveToBalanceCommunity).mirrorY();
            driveToBalanceOutside = new Vector(driveToBalanceOutside).mirrorY();

            driveToBalanceAngle[0] = -driveToBalanceAngle[0];
        }

        sg.addCommands(new InstantCommand(() -> r.arm.setArmOffset(AutonPos.initArmAngle, AutonPos.initArmStendo)));
        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotLocation(startPositions[startPos])));
        sg.addCommands(new InstantCommand(() -> r.sensors.resetNavXAng(startAng[startPos])));

        startAngle = startAng[startPos];
        System.out.println("start angle: " + startAngle);

        //Do nothing command
        if(selectedAuton == 0) {
            return sg;
        }

        //Score
        if(selectedAuton != 0 && selectedAuton != 1){
            sg.addCommands(scoreOnlyCone(r));
        }

        //First Drive Out
        if(selectedAuton == 1 || selectedAuton == 2){
            sg.addCommands(new AngleMotionProfile(r, driveOutOneAng[startPos]));
            /*if(startPos == 1 || startPos == 2){
                sg.addCommands(new DriveMotionProfile(r, driveOutOne[startPos], driveOutOneAng[startPos], mpCalsThere[startPos]));
            }*/
        }

        //Drive Out Only
        if(selectedAuton == 1 || selectedAuton == 2){
            if(startPos == 0 || startPos == 3){
                sg.addCommands(new DriveMotionProfile(r, driveOutTwo[startPos], driveOutOneAng[startPos], mpCalsThere[startPos]));
            } else {
                sg.addCommands(new DriveForTime(r, Vector.fromDeg(0.2, 0), 0, 6.0));
            }
        }
        
        //Piece Gather
        if(selectedAuton == 4 || selectedAuton == 5 || selectedAuton == 6){
            if(startPos == 1 || startPos == 2){
                sg.addCommands(new AngleMotionProfile(r, driveOutOneAng[startPos]));
                sg.addCommands(new InstantCommand(() -> r.driveTrain.targetHeading = driveOutOneAng[startPos]));
                sg.addCommands(new InstantCommand(r.gripper::open));//because this takes too long, do it early
                sg.addCommands(AutoBalance.getDriveOverStation(r, false));
            }
            sg.addCommands(new AngleMotionProfile(r, driveToPieceAng[startPos]));
            sg.addCommands(new DriveMotionProfile(r, driveToPiece[startPos], driveToPieceAng[startPos], mpCalsThere[startPos])
                .raceWith(GatherCommand.gatherCommand(r)));
            sg.addCommands(new ArmGoHome(r));
        }

        //Score #2
        if(selectedAuton == 5 || selectedAuton == 6){
            
            sg.addCommands(new AngleMotionProfile(r, driveToScoreAng[startPos]));
            if(startPos == 1 || startPos == 2){
                sg.addCommands(new DriveMotionProfile(r, driveToBalanceOutside, driveToScoreAng[startPos]));
                sg.addCommands(AutoBalance.getDriveOverStation(r, true));
            }
            DriveMotionProfile dmp = new DriveMotionProfile(r, driveToScore[startPos], driveToScoreAng[startPos], mpCalsBack[startPos]);
            sg.addCommands(dmp.alongWith(new NegativeWait(1.5, dmp).andThen(new ArmMove(r, r.arm.cals.positionCubeHi))));
            sg.addCommands(scoreOnlyCube(r));
            if(selectedAuton == 5){
                sg.addCommands(new AngleMotionProfile(r, 0));
            }
        }

        //Balance
        if(selectedAuton >= 3 && selectedAuton != 5){
            final Vector SELECTED_DRIVE_TO_BALANCE;
            if(selectedAuton == 4){
                SELECTED_DRIVE_TO_BALANCE = driveToBalanceOutside;
            } else {
                SELECTED_DRIVE_TO_BALANCE = driveToBalanceCommunity;
            }

            sg.addCommands(new AngleMotionProfile(r, driveToBalanceAngle[0]));
            if(selectedAuton == 4){
                sg.addCommands(new DriveMotionProfile(r, SELECTED_DRIVE_TO_BALANCE, driveToBalanceAngle[0]));
                sg.addCommands(new InstantCommand(() -> r.driveTrain.targetHeading = driveToBalanceAngle[0]));
            } else {
                sg.addCommands(AutoBalance.getDriveOverStation(r, false));
                sg.addCommands(new DriveForTime(r, Vector.fromDeg(0.2, 0), 0.8));
            }
            sg.addCommands(AutoBalance.getAutoBalanceCommand(r, true));
        }

        //Shoot Cube
        if(selectedAuton == 4){
            sg.addCommands(launchThatCubeBaby(r));
        }
        
        return sg;
    }

    public static Command scoreOnlyCone(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();
        
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
        sg.addCommands((new DriveForTime(r, Vector.fromXY(0.25, 0), 0.6)).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(-.3))));
        //lower arm
        sg.addCommands(new ArmGoHome(r).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));

        return sg;
    }

    public static Command scoreOnlyCube(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        //eject it
        sg.addCommands(new RunCommand(() -> r.gripper.setIntakePower(r.gripper.cals.cubeScorePower), r.gripper).raceWith(new WaitCommand(0.2)));
        sg.addCommands(new PrintCommand("CubeLaunched"));
        //drive backwards
        sg.addCommands(new DriveForTime(r, Vector.fromXY(0.25, 0), 0.4));
        //lower arm
        sg.addCommands(new ArmGoHome(r).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));

        return sg;
    }

    public static Command launchThatCubeBaby(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        //set the arm to launch position
        sg.addCommands(new ArmMove(r, Vector.fromDeg(35, 120)));
        //eject it
        sg.addCommands(new RunCommand(() -> r.gripper.setIntakePower(-0.8), r.gripper).raceWith(new WaitCommand(0.2)));
        sg.addCommands(new PrintCommand("CubeLaunched"));
        //lower arm
        sg.addCommands(new ArmGoHome(r).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));

        return sg;
    }
}
