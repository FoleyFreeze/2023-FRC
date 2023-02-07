package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AdvancedMovement.DriveMotionProfile;
import frc.robot.commands.Auton.AutonToolbox.AutoBalance;
import frc.robot.commands.Auton.AutonToolbox.SetStartPos;
import frc.robot.commands.Auton.BasicMovement.DistanceDrive;

public class AutonBuilder {

    /* This function is stupid and long but 
     * I don't know what to do about it let's
     * hope it just works the first time and
     * I don't have to debug this too much :()
     */
    public static Command buildAuton(RobotContainer r, int startPosChooser, boolean secondPieceChooser, int actionChooser, int pathChooser, int pieceChooser){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        sg.addCommands(new SetStartPos(r, AutonPos.START_POSITIONS[startPosChooser]));

        if(actionChooser != 0){
            if(actionChooser != 1){
                sg.addCommands(/*Score Command*/);
            }

            if(actionChooser == 1 || actionChooser == 2){//This just moves the bot out of the community with a basic command
                sg.addCommands(new DistanceDrive(r, AutonPos.FIRST_DRIVE[pathChooser]));
                sg.addCommands(new DistanceDrive(r, AutonPos.JUST_DRIVE_OUT[pathChooser]));
            }

            if(actionChooser == 3){//This is the auto-balance after a one-ball auton
                sg.addCommands(new DriveMotionProfile(r, AutonPos.FIRST_DRIVE[1].xy));
                sg.addCommands(AutoBalance.getAutoBalanceCommand(r));
            }

            if(actionChooser == 4 || actionChooser == 5){//Goes to midfield, gathers, and goes back to score. We need to somehow combine the drive and gather if we want it to be fast
                sg.addCommands(new DriveMotionProfile(r, AutonPos.FIRST_DRIVE[pathChooser].xy));
                sg.addCommands(new DriveMotionProfile(r, AutonPos.MID_FIELD_POS[pieceChooser].xy));

                sg.addCommands(/*Gather Command*/);

                int secondScorePos = startPosChooser;
                if(secondPieceChooser){//This determines the second score position. IDK man, this could probably be done some better way, but this is all I got
                    secondScorePos++;
                    if(secondScorePos > 8){
                        secondScorePos = 7;//We default to the other way if the way we chose is not possible
                    }
                } else {
                    secondScorePos --;
                    if(secondScorePos < 0){
                        secondScorePos = 1;//We default to the other way if the way we chose is not possible
                    }
                }

                sg.addCommands(new DriveMotionProfile(r, AutonPos.FIRST_DRIVE[pathChooser].xy));
                sg.addCommands(new DriveMotionProfile(r, AutonPos.START_POSITIONS[secondScorePos].xy));

                sg.addCommands(/*Score Command*/);
            }

            if(actionChooser == 5){//This is the auto-balance after a two-ball auton
                sg.addCommands(new DriveMotionProfile(r, AutonPos.FIRST_DRIVE[1].xy));
                sg.addCommands(AutoBalance.getAutoBalanceCommand(r));
            }
        }

        return sg;
    }
}
