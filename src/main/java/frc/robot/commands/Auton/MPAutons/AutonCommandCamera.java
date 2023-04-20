package frc.robot.commands.Auton.MPAutons;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.AutonPaths;
import frc.robot.RobotContainer.AutonStarts;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AdvancedMovement.AngleMotionProfile;
import frc.robot.commands.Auton.AutonToolbox.AutoBalance;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.commands.Combos.CamCommands;
import frc.robot.util.Vector;

public class AutonCommandCamera {

    public static Command autonCamCommand(RobotContainer r, Alliance team, AutonPaths path, AutonStarts startPos){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        Vector[] startPositions = {AutonPos.substation.xy,
                                   AutonPos.subMid.xy,
                                   AutonPos.farMid.xy,
                                   AutonPos.far.xy};
        double[] startAng = {AutonPos.substation.value,
                             AutonPos.subMid.value,
                             AutonPos.farMid.value,
                             AutonPos.far.value};

        Vector[] driveToPiece = {Vector.fromDeg(0.65, 5),
                                 Vector.fromDeg(0.0, 0.0),
                                 Vector.fromDeg(0.0, 0.0),
                                 Vector.fromDeg(0.5, -2.0)};
        double[] driveToPieceTime = {1.3, 0.0, 0.0, 1.45};

        Vector[] piecePositions = {Vector.fromXY(278.3, 36.19 + 48*3),
                                   Vector.fromXY(278.3, 36.19 + 48*2),
                                   Vector.fromXY(278.3, 36.19 + 48),
                                   Vector.fromXY(278.3, 36.19)};

        Vector driveBackBumpOne = Vector.fromDeg(0.5, -179);
        Vector driveBackBumpTwo = Vector.fromDeg(0.3, 180);

        Vector driveBackClear = Vector.fromDeg(0.65, 179);

        Vector driveBackBalance[] = {Vector.fromDeg(0.0, 0),
                                     Vector.fromDeg(0.55, -175),
                                     Vector.fromDeg(0.55, 175),
                                     Vector.fromDeg(0.0, 0)};

        double[] driveToPieceAng = {0, 15, -15, 0};
        double[] driveBackAng = {180, 180, 180, 172};

        int[] scorePositionsRed = {26, 23, 23, 20};
        int[] scorePositionsBlue = {20, 23, 23, 26};

        int scorePosition;

        if(team == Alliance.Red){
            for(int i = 0; i < 4; i++){
                startPositions[i] = new Vector(startPositions[i]).mirrorY();
                startAng[i] = -startAng[i];

                driveToPiece[i] = new Vector(driveToPiece[i]).mirrorY();

                piecePositions[i] = new Vector(piecePositions[i]).mirrorY();

                driveToPieceAng[i] = -driveToPieceAng[i];
                driveBackAng[i] = -driveBackAng[i];

                driveBackBalance[i].mirrorY();
            }

            driveBackBumpOne.mirrorY();
            driveBackBumpTwo.mirrorY();

            driveBackClear.mirrorY();

            scorePosition = scorePositionsRed[startPos.ordinal()];
        } else {
            scorePosition = scorePositionsBlue[startPos.ordinal()];
        }

        sg.addCommands(new InstantCommand(() -> r.arm.setArmOffset(AutonPos.initArmAngle, AutonPos.initArmStendo)));//init arm
        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotLocation(startPositions[startPos.ordinal()])));//init location
        sg.addCommands(new InstantCommand(() -> r.sensors.resetNavXAng(startAng[startPos.ordinal()])));//init angle

        sg.addCommands(new InstantCommand(() -> r.inputs.setAutonScorePosition(scorePosition)));//set the second score position

        //score the first piece
        sg.addCommands(scoreOnlyCone(r));

        //get over the charge station if you're in the middle
        if(startPos == AutonStarts.FAR_MID || startPos == AutonStarts.SUB_MID){
            sg.addCommands(new DriveForTime(r, Vector.fromDeg(0.5, 0), 180, 0.2));
            sg.addCommands(AutoBalance.getDriveOverStation(r, false, 180).alongWith(new InstantCommand(() -> r.gripper.open())));
        }

        //drive for time and pickup the piece
        sg.addCommands(new DriveForTime(r, driveToPiece[startPos.ordinal()], 180, driveToPieceTime[startPos.ordinal()]));
        //profile to an angle
        sg.addCommands(new AngleMotionProfile(r, Math.toRadians(driveToPieceAng[startPos.ordinal()])).alongWith(new InstantCommand(() -> r.gripper.open())));
        if(startPos == AutonStarts.FAR || startPos == AutonStarts.SUB){
            sg.addCommands(CamCommands.AutoPickup(r, piecePositions[startPos.ordinal()]));
        } else {
            sg.addCommands(CamCommands.AutoPickup(r, piecePositions[startPos.ordinal()]).raceWith(new WaitCommand(2.0)));
        }


        //set the gripper and send the arm back home
        sg.addCommands(new InstantCommand(() -> r.gripper.setIntakePower(0.07)));
        sg.addCommands(new ArmGoHome(r));

        //turn around
        sg.addCommands((new AngleMotionProfile(r, Math.toRadians(driveBackAng[startPos.ordinal()]))));

        //score &| balance
        if(startPos == AutonStarts.FAR_MID || startPos == AutonStarts.SUB_MID){
            if(r.sensors.odo.botLocation.getX() > 193.25 + 13.0 + 18.0){
                sg.addCommands(new DriveForTime(r, driveBackBalance[startPos.ordinal()], driveBackAng[startPos.ordinal()], 0.6));
            }
            sg.addCommands(new InstantCommand(() -> r.driveTrain.targetHeading = Math.toRadians(180)));
            sg.addCommands(AutoBalance.getAutoBalanceCommand(r, true)
                    .alongWith(new WaitCommand(15).until(() -> r.sensors.getAbsPitchRoll() > 20)
                        .andThen(new ArmMove(r, Vector.fromDeg(35,85), true))));
            sg.addCommands(AutonCommand.launchThatCubeBaby(r));
        } else {
            if(startPos == AutonStarts.FAR){//drive back slower if you're bump side
                sg.addCommands(new DriveForTime(r, driveBackBumpOne, driveBackAng[startPos.ordinal()], 1.0));
                sg.addCommands(new DriveForTime(r, driveBackBumpTwo, driveBackAng[startPos.ordinal()], 1.3));
            } else {
                sg.addCommands(new DriveForTime(r, driveBackClear, driveBackAng[startPos.ordinal()], 1.0));
            }
            sg.addCommands(CamCommands.AutoDriveToScore(r));
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
        sg.addCommands((new DriveForTime(r, Vector.fromXY(0.5, 0), 0.35)).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(-.3))));
        //lower arm
        sg.addCommands(new ArmGoHome(r, true).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));

        return sg;
    }
}
