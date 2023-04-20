package frc.robot.commands.Auton.AutonToolbox;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AdvancedMovement.FancyMotionProfile;
import frc.robot.commands.Auton.AdvancedMovement.Tag;
import frc.robot.commands.Auton.MPAutons.AutonCommandCamera;
import frc.robot.util.Vector;

public class FancyAutons {
    
    public static Command threePiece(RobotContainer r, boolean bump, Alliance alliance){

        System.out.println("Creating Fancy Auton Command");
        SequentialCommandGroup sg = new SequentialCommandGroup();

        AutonPos startPos = getStartPos(bump);

        Vector firstDriveOut = Vector.addVectors(startPos.xy,Vector.fromXY(30,0));

        Vector pieceOne = getPieceOnePos(bump);

        if(alliance.equals(Alliance.Red)){
            startPos.xy.mirrorY();
            startPos.value = -startPos.value;

            firstDriveOut.mirrorY();

            pieceOne.mirrorY();
        }

        ArrayList<Vector> wps = new ArrayList<>();
        wps.add(firstDriveOut);//drive out
        wps.add(pieceOne);

        ArrayList<Tag> tags = new ArrayList<>();
        //tags.add(Tag.)

        sg.addCommands(new InstantCommand(() -> r.arm.setArmOffset(AutonPos.initArmAngle, AutonPos.initArmStendo)));
        sg.addCommands(new InstantCommand(() -> r.sensors.odo.setBotLocation(startPos.xy)));
        sg.addCommands(new InstantCommand(() -> r.sensors.resetNavXAng(startPos.value)));

        sg.addCommands(AutonCommandCamera.scoreOnlyCone(r));

        sg.addCommands(new FancyMotionProfile(r, AutonCal.driveBase, wps, tags));

        return sg;
    }


    private static AutonPos getStartPos(boolean bump){
        if(bump){
            return new AutonPos(AutonPos.far);
        } else {
            return new AutonPos(AutonPos.substation);
        }
    }

    private static Vector getPieceOnePos(boolean bump){
        if(bump){
            return new Vector(AutonPos.drivePieceFar.xy);
        } else {
            return new Vector(AutonPos.drivePieceSub.xy);
        }
    }
}
