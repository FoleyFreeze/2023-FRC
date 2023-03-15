package frc.robot.commands.Auton.AutonToolbox;

import java.time.Instant;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.BasicMovement.DistanceDrive;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.commands.Gripper.GatherCommand;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class AutonCommand {
    public static Command autonCommand(RobotContainer r, Alliance alliance, int selectedAuton, int startPos){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        Vector[] startPositions = {Vector.fromXY(0, 0),
                                   Vector.fromXY(0, 0),
                                   Vector.fromXY(0, 0),
                                   Vector.fromXY(0, 0)};
        double[] startAng = {165, 165, 165, 165};

        Vector[] driveOutOne = {Vector.fromXY(0, 0),
                                Vector.fromXY(0, 30),
                                Vector.fromXY(0, -30),
                                Vector.fromXY(0, 0)};
        double[] driveOutOneAng = {0, 0, 0, 0};

        Vector[] driveOutTwo = {Vector.fromXY(85, 0),
                                Vector.fromXY(85, 0),
                                Vector.fromXY(85, 0),
                                Vector.fromXY(85, 0)};
        double[] driveOutTwoAng = {0, 0, 0, 0};

        Vector[] driveToPiece = {Vector.fromXY(0, 180.2),
                                 Vector.fromXY(0, 180.2),
                                 Vector.fromXY(0, 36.2),
                                 Vector.fromXY(0, 36.2)};
        double[] driveToPieceAng = {0, 0, 0, 0};

        Vector[] driveToScore = {Vector.fromXY(-150, 0),
                                 Vector.fromXY(-150, 0),
                                 Vector.fromXY(-150, 0),
                                 Vector.fromXY(-150, 0)};
        double[] driveToScoreAng = {180, 180, 180, 180};

        Vector[] driveToBalanceCommunity = {Vector.fromXY(0, -52),
                                            Vector.fromXY(0, -24),
                                            Vector.fromXY(0, 24),
                                            Vector.fromXY(0, 52)};
        Vector[] driveToBalanceOutside = {Vector.fromXY(-12, -52),
                                          Vector.fromXY(-12, -52),
                                          Vector.fromXY(-12, 52),
                                          Vector.fromXY(-12, 52)};
        double driveToBalanceAngle = 90;

        if(alliance.equals(Alliance.Red)){
            for(int i = 0; i < 4; i++){
                startPositions[i].theta = -startPositions[i].theta;
                startAng[i] = -startAng[i];

                driveOutOne[i].theta = -driveOutOne[i].theta;
                driveOutOneAng[i] = -driveOutOneAng[i];

                driveOutTwo[i].theta = -driveOutTwo[i].theta;
                driveOutTwoAng[i] = -driveOutTwoAng[i];

                driveToPiece[i].theta = -driveToPiece[i].theta;
                driveToPieceAng[i] = -driveToPieceAng[i];

                driveToBalanceCommunity[i].theta = -driveToBalanceCommunity[i].theta;
                driveToBalanceOutside[i].theta = -driveToBalanceOutside[i].theta;
            }
            driveToBalanceAngle = -driveToBalanceAngle;
        }

        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotLocation(startPositions[startPos])));
        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotAngle(Angle.toRad(startAng[startPos]))));

        //Do nothing command
        if(selectedAuton == 0) {
            return sg;
        }

        //Score
        if(selectedAuton != 0 && selectedAuton != 1){
            sg.addCommands(scoreOnlyCone(r));
        }

        //First Drive Out
        if(selectedAuton == 1 || selectedAuton == 4 || selectedAuton == 5){
            sg.addCommands(new DistanceDrive(r, driveOutOne[startPos], driveOutOneAng[startPos])
                .alongWith(new WaitCommand(1.0)
                .andThen(new ArmGoHome(r))
                .alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0)))));
        }

        //Drive Out Only
        if(selectedAuton == 2){
            sg.addCommands(new DistanceDrive(r, driveOutTwo[startPos], driveOutTwoAng[startPos]));
        }
        
        //Piece Gather
        if(selectedAuton == 4 || selectedAuton == 5){
            sg.addCommands(new DistanceDrive(r, driveToPiece[startPos])
                .alongWith(GatherCommand.gatherCommand(r)));
        }

        //Score #2
        if(selectedAuton == 5){
            sg.addCommands(new DistanceDrive(r, driveToScore[startPos], driveToScoreAng[startPos])
                .alongWith(new WaitCommand(1.0).andThen(new ArmMove(r, r.arm.cals.positionCubeHi)))
                .andThen(scoreOnlyCube(r)));
        }

        //Balance
        if(selectedAuton >= 3){
            final Vector SELECTED_DRIVE_TO_BALANCE;
            if(selectedAuton == 4){
                SELECTED_DRIVE_TO_BALANCE = driveToBalanceCommunity[startPos];
            } else {
                SELECTED_DRIVE_TO_BALANCE = driveToBalanceCommunity[startPos];
            }
            sg.addCommands(new DriveForTime(r, SELECTED_DRIVE_TO_BALANCE, driveToBalanceAngle)
                .alongWith(new ConditionalCommand(
                    new WaitCommand(1.0).andThen(new ArmGoHome(r)), 
                    new WaitCommand(0.0), () -> selectedAuton == 3))
                .alongWith(new InstantCommand(() -> r.gripper.setIntakePower(0))));
            sg.addCommands(AutoBalance.getAutoBalanceCommand(r));
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
        //move the arm to hi release position
        sg.addCommands(new ArmMove(r, r.arm.cals.positionConeHiRelease));
        //driver backwards and spin gripper backwards
        sg.addCommands((new DistanceDrive(r, Vector.fromXY(15, 0))).alongWith(new InstantCommand(() -> r.gripper.setIntakePower(-.3))));
    
        return sg;
    }

    public static Command scoreOnlyCube(RobotContainer r){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        //eject it
        sg.addCommands(new RunCommand(() -> r.gripper.setIntakeSpeed(r.gripper.cals.cubeScoreSpeed), r.gripper));
        //drive backwards
        sg.addCommands(new DistanceDrive(r, Vector.fromXY(15, 0)));

        return sg;
    }
}
