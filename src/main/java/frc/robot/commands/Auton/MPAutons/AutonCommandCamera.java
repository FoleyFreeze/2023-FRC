package frc.robot.commands.Auton.MPAutons;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AdvancedMovement.DriveMotionProfile;
import frc.robot.commands.Auton.AutonCal.MPCals;
import frc.robot.commands.Combos.CamCommands;
import frc.robot.util.Vector;

public class AutonCommandCamera {

    public static Command autonPiecePickup(RobotContainer r, int startPos){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        Vector[] driveToPieceCam = {AutonPos.drivePieceMidSubCam.xy,
                                    AutonPos.drivePieceMidSubCam.xy,
                                    AutonPos.drivePieceMidFarCam.xy,
                                    AutonPos.drivePieceFarCam.xy};
        double[] driveToPieceAngCam = {AutonPos.drivePieceMidSubCam.value,
                                       AutonPos.drivePieceMidSubCam.value,
                                       AutonPos.drivePieceMidFarCam.value,
                                       AutonPos.drivePieceFarCam.value};

        MPCals[] mpCalsThere = {AutonCal.driveBase,
                                AutonCal.driveBase,
                                AutonCal.driveBase,
                                AutonCal.bumpThereCals};


        if(DriverStation.getAlliance() == Alliance.Red){
            for(int i = 0; i < startPos; i++){
                driveToPieceCam[i] = new Vector(driveToPieceCam[i]).mirrorY();
                driveToPieceAngCam[i] = -driveToPieceAngCam[i];
            }
        }

        //Motion profile to a spot and then drive to pickup the game piece
        sg.addCommands((new DriveMotionProfile(r, driveToPieceCam[startPos], driveToPieceAngCam[startPos], mpCalsThere[startPos])));
        sg.addCommands(CamCommands.AutoPickup(r).raceWith(new WaitCommand(2.0)));


        //set the gripper and send the arm back home
        sg.addCommands(new InstantCommand(() -> r.gripper.setIntakePower(0.07)));
        sg.addCommands(new ArmGoHome(r));
        return sg;
    }
}
