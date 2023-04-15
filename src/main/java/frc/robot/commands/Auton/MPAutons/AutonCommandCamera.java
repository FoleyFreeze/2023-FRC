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
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AdvancedMovement.AngleMotionProfile;
import frc.robot.commands.Auton.AutonToolbox.AutoBalance;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.commands.Combos.CamCommands;
import frc.robot.util.Vector;

public class AutonCommandCamera {

    public static Command autonCamCommand(RobotContainer r, Alliance team, AutonStarts startPos){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        Vector[] startPositions = {AutonPos.substation.xy,
                                   AutonPos.subMid.xy,
                                   AutonPos.farMid.xy,
                                   AutonPos.far.xy};
        double[] startAng = {AutonPos.substation.value,
                             AutonPos.subMid.value,
                             AutonPos.farMid.value,
                             AutonPos.far.value};

        Vector[] driveToPiece = {Vector.fromDeg(0.6, 0.0),
                                 Vector.fromDeg(0.0, 0.0),
                                 Vector.fromDeg(0.0, 0.0),
                                 Vector.fromDeg(0.4, 0.0)};
        double[] driveToPieceTime = {1.5, 0.0, 0.0, 2.0};

        double driveToPieceAng = Math.toRadians(0);
        double driveBackAng = Math.toRadians(180);

        int[] scorePositionsRed = {26, 26, 20, 20};
        int[] scorePositionsBlue = {20, 20, 26, 26};

        int scorePosition;

        if(DriverStation.getAlliance() == Alliance.Red){
            for(int i = 0; i < 4; i++){
                startPositions[i] = new Vector(startPositions[i]).mirrorY();
                startAng[i] = -startAng[i];

                driveToPiece[i] = new Vector(driveToPiece[i]).mirrorY();
            }
            scorePosition = scorePositionsRed[startPos.ordinal()];
        } else {
            scorePosition = scorePositionsBlue[startPos.ordinal()];
        }

        sg.addCommands(new InstantCommand(() -> r.arm.setArmOffset(AutonPos.initArmAngle, AutonPos.initArmStendo)));//init arm
        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotLocation(startPositions[startPos.ordinal()])));//init location
        sg.addCommands(new InstantCommand(() -> r.sensors.resetNavXAng(startAng[startPos.ordinal()])));//init angle

        sg.addCommands(new InstantCommand(() -> r.inputs.setAutonScorePosition(scorePosition)));

        //score the first piece
        sg.addCommands(AutonCommand.scoreOnlyCone(r));

        //profile to an angle
        sg.addCommands(new AngleMotionProfile(r, driveToPieceAng));

        //get over the charge station if you're in the middle
        if(startPos == AutonStarts.FAR_MID || startPos == AutonStarts.SUB_MID){
            AutoBalance.getDriveOverStation(r, false);
        }

        //drive for time and pickup the piece
        sg.addCommands(new DriveForTime(r, driveToPiece[startPos.ordinal()], driveToPieceAng, driveToPieceTime[startPos.ordinal()]));
        sg.addCommands(CamCommands.AutoPickup(r).raceWith(new WaitCommand(2.0)));


        //set the gripper and send the arm back home
        sg.addCommands(new InstantCommand(() -> r.gripper.setIntakePower(0.07)));
        sg.addCommands(new ArmGoHome(r));

        //turn around
        sg.addCommands((new AngleMotionProfile(r, driveBackAng)));

        //score &| balance
        if(startPos == AutonStarts.FAR_MID || startPos == AutonStarts.SUB_MID){
            sg.addCommands(AutoBalance.getAutoBalanceCommand(r, true));
            sg.addCommands(AutonCommand.launchThatCubeBaby(r));
        } else {
            sg.addCommands(CamCommands.AutoDriveToScore(r));
        }

        return sg;
    }
}
