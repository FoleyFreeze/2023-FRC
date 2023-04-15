package frc.robot.commands.Auton.MPAutons;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.AutonPaths;
import frc.robot.RobotContainer.AutonStarts;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AdvancedMovement.AngleMotionProfile;
import frc.robot.commands.Auton.AdvancedMovement.DriveMotionProfile;
import frc.robot.commands.Auton.AutonCal.MPCals;
import frc.robot.commands.Auton.AutonToolbox.AutoBalance;
import frc.robot.commands.Auton.AutonToolbox.NegativeWait;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.commands.Gripper.GatherCommand;
import frc.robot.util.Vector;

public class AutonCommand {

    public static double startAngle = 0;
    public static Command autonCommand(RobotContainer r, Alliance alliance, AutonPaths selectedAuton, AutonStarts startPos){
        
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
        Vector[] driveToPieceRedOffset = {Vector.fromXY(0, 9+1),
                                          Vector.fromXY(0, 0),
                                          Vector.fromXY(0, 0),
                                          Vector.fromXY(0, -4)};
        Vector[] driveToPieceBlueOffset = {Vector.fromXY(0, 8),
                                            Vector.fromXY(0, 0),
                                            Vector.fromXY(0, 0),
                                            Vector.fromXY(0, -4)};

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
                                AutonCal.driveBase,
                                AutonCal.bumpThereCals};

        MPCals[] mpCalsBack = {AutonCal.driveBase,
                            AutonCal.driveBase,
                            AutonCal.driveBase,
                            AutonCal.bumpBackCals};

        

        Vector driveToBalanceCommunity = new Vector(AutonPos.driveToBalComm.xy);
        Vector driveToBalanceOutside = new Vector(AutonPos.driveToBalOutside.xy);
        double[] driveToBalanceAngle = {AutonPos.driveToBalComm.value}; //doing this to make it a reference

        if(alliance.equals(Alliance.Red)){
            for(int i = 0; i < 4; i++){
                //System.out.println("dtp was " + driveToPiece[i]);
                driveToPiece[i] = Vector.addVectors(driveToPieceRedOffset[i],driveToPiece[i]);
                //System.out.println("dtp is now " + driveToPiece[i]);

                startPositions[i] = new Vector(startPositions[i]).mirrorY();
                startAng[i] = -startAng[i];

                driveOutOneAng[i] = -driveOutOneAng[i];

                driveOutTwo[i] = new Vector(driveOutTwo[i]).mirrorY();

                driveToPiece[i] = new Vector(driveToPiece[i]).mirrorY();
                driveToPieceAng[i] = -driveToPieceAng[i];

                driveToScore[i] = new Vector(driveToScore[i]).mirrorY();
                driveToScoreAng[i] = -driveToScoreAng[i];

                //System.out.println("flipped dtp is now " + driveToPiece[i]);
                //System.out.println("flipped path is " + Vector.subVectors(driveToPiece[i], startPositions[i]));
            }
            driveToBalanceCommunity = new Vector(driveToBalanceCommunity).mirrorY();
            driveToBalanceOutside = new Vector(driveToBalanceOutside).mirrorY();

            driveToBalanceAngle[0] = -driveToBalanceAngle[0];
        } else {
            for(int i = 0; i < 4; i++){
                driveToPiece[i] = Vector.addVectors(driveToPieceBlueOffset[i],driveToPiece[i]);
            }
        }

        sg.addCommands(new InstantCommand(() -> r.arm.setArmOffset(AutonPos.initArmAngle, AutonPos.initArmStendo)));
        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotLocation(startPositions[startPos.ordinal()])));
        sg.addCommands(new InstantCommand(() -> r.sensors.resetNavXAng(startAng[startPos.ordinal()])));

        startAngle = startAng[startPos.ordinal()];
        System.out.println("start angle: " + startAngle);

        //Do nothing command
        if(selectedAuton == AutonPaths.NOTHING) {
            return sg;
        }

        //Score
        if(selectedAuton != AutonPaths.NOTHING && selectedAuton != AutonPaths.DRIVE_OUT){
            sg.addCommands(scoreOnlyCone(r));
        }

        //First Drive Out
        if(selectedAuton == AutonPaths.DRIVE_OUT || selectedAuton == AutonPaths.SCORE_DRIVE_OUT){
            sg.addCommands(new AngleMotionProfile(r, driveOutOneAng[startPos.ordinal()]));
        }

        //Drive Out Only
        if(selectedAuton == AutonPaths.DRIVE_OUT || selectedAuton == AutonPaths.SCORE_DRIVE_OUT){
            if(startPos == AutonStarts.SUB || startPos == AutonStarts.FAR){
                //drive out
                sg.addCommands(new DriveMotionProfile(r, driveOutTwo[startPos.ordinal()], driveOutOneAng[startPos.ordinal()], mpCalsThere[startPos.ordinal()]));
            } else {
                //drive only over charge station
                sg.addCommands(AutoBalance.getDriveOverStation(r, false).andThen(new DriveForTime(r, Vector.fromDeg(0.2, 0), 0.9)));
            }
        }
        
        //Piece Gather
        if(selectedAuton.ordinal() >= 4){
            if(startPos == AutonStarts.SUB_MID || startPos == AutonStarts.FAR_MID){
                //get over the charge station
                sg.addCommands(new AngleMotionProfile(r, driveOutOneAng[startPos.ordinal()]));
                sg.addCommands(new InstantCommand(() -> r.driveTrain.targetHeading = driveOutOneAng[startPos.ordinal()]));
                sg.addCommands(new InstantCommand(r.gripper::open));//because this takes too long, do it early
                sg.addCommands(AutoBalance.getDriveOverStation(r, false));
            }
            if(false){
                sg.addCommands(AutonCommandCamera.autonPiecePickup(r, startPos.ordinal()));
            } else {
                //drive to gather angle
                sg.addCommands(new AngleMotionProfile(r, driveToPieceAng[startPos.ordinal()]));
                //drive to gather
                sg.addCommands(new DriveMotionProfile(r, driveToPiece[startPos.ordinal()], driveToPieceAng[startPos.ordinal()], mpCalsThere[startPos.ordinal()])
                    .raceWith(GatherCommand.gatherCommand(r)));
                //send arm home
                sg.addCommands(new ArmGoHome(r));
            }
        }

        //Score #2
        if(selectedAuton == AutonPaths.TWO_SCORE || selectedAuton == AutonPaths.TWO_SCORE_BALANCE){
            
            sg.addCommands(new AngleMotionProfile(r, driveToScoreAng[startPos.ordinal()]));
            if(startPos.ordinal() == 1 || startPos.ordinal() == 2){
                sg.addCommands(new DriveMotionProfile(r, driveToBalanceOutside, driveToScoreAng[startPos.ordinal()]));
                sg.addCommands(AutoBalance.getDriveOverStation(r, true));
            }
            DriveMotionProfile dmp = new DriveMotionProfile(r, driveToScore[startPos.ordinal()], driveToScoreAng[startPos.ordinal()], mpCalsBack[startPos.ordinal()]);
            sg.addCommands(dmp.alongWith(new NegativeWait(1.5, dmp).andThen(new ArmMove(r, r.arm.cals.positionCubeHi))));
            sg.addCommands(scoreOnlyCube(r));
            if(selectedAuton == AutonPaths.TWO_SCORE){
                sg.addCommands(new AngleMotionProfile(r, 0));
            }
        }

        //Balance
        if(selectedAuton == AutonPaths.SCORE_PICKUP_BALANCE_TOSS && startPos == AutonStarts.FAR_MID){
            //if we are doing 2cube over the charge station close to bump side
            sg.addCommands(new AngleMotionProfile(r, driveToBalanceAngle[0])); //point at the driverstation
            sg.addCommands(new InstantCommand(() -> r.driveTrain.targetHeading = driveToBalanceAngle[0]));
            sg.addCommands(new DriveForTime(r, Vector.fromXY(-0.4,0), 0.3));
            sg.addCommands(AutoBalance.getAutoBalanceCommand(r, true)
                    .alongWith(new WaitCommand(15).until(() -> r.sensors.getAbsPitchRoll() > 20)
                        .andThen(new ArmMove(r, Vector.fromDeg(35,75), true))));
            

        } else if(selectedAuton.ordinal() >= 3 && selectedAuton != AutonPaths.TWO_SCORE){
            final Vector SELECTED_DRIVE_TO_BALANCE;
            if(selectedAuton == AutonPaths.SCORE_PICKUP_BALANCE || selectedAuton == AutonPaths.SCORE_PICKUP_BALANCE_TOSS){
                SELECTED_DRIVE_TO_BALANCE = driveToBalanceOutside;
            } else {
                SELECTED_DRIVE_TO_BALANCE = driveToBalanceCommunity;
            }

            sg.addCommands(new AngleMotionProfile(r, driveToBalanceAngle[0]));
            if(selectedAuton == AutonPaths.SCORE_PICKUP_BALANCE || selectedAuton == AutonPaths.SCORE_PICKUP_BALANCE_TOSS){
                sg.addCommands(new DriveMotionProfile(r, SELECTED_DRIVE_TO_BALANCE, driveToBalanceAngle[0]));
                sg.addCommands(new InstantCommand(() -> r.driveTrain.targetHeading = driveToBalanceAngle[0]));
            } else {
                sg.addCommands(AutoBalance.getDriveOverStation(r, false));
                sg.addCommands(new DriveForTime(r, Vector.fromDeg(0.2, 0), 0.8));
            }

            if(selectedAuton == AutonPaths.SCORE_PICKUP_BALANCE_TOSS){
                //also set the arm to launch position
                sg.addCommands(AutoBalance.getAutoBalanceCommand(r, true)
                    .alongWith(new WaitCommand(15).until(() -> r.sensors.getAbsPitchRoll() > 20)
                        .andThen(new ArmMove(r, Vector.fromDeg(35,75)))));
            } else {
                sg.addCommands(AutoBalance.getAutoBalanceCommand(r, true));
            }
        }

        //Shoot Cube
        if(selectedAuton == AutonPaths.SCORE_PICKUP_BALANCE_TOSS){
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
        //sg.addCommands(new ArmMove(r, Vector.fromDeg(35, 120)));
        //eject it
        sg.addCommands(new RunCommand(() -> r.gripper.setIntakePower(-0.8), r.gripper).raceWith(new WaitCommand(0.25)));
        sg.addCommands(new PrintCommand("CubeLaunched"));
        //lower arm
        sg.addCommands(new ArmGoHome(r).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));

        return sg;
    }

    public static Command cameraAutonCommand(RobotContainer r, Alliance alliance, AutonPaths selectedAuton, AutonStarts startPos){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        return sg;
    }
}
