package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AdvancedMovement.MultiDimensionalMotionProfile;
import frc.robot.commands.Auton.BasicMovement.DriveForTime;
import frc.robot.commands.Gripper.GatherCommand;
import frc.robot.subsystems.Inputs.Inputs.Level;
import frc.robot.util.Vector;

public class Score extends CommandBase{

    /* THIS CLASS IS TOTALLY OBSOLETE AND DOES NOTHING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     *
     * 
     * 
     * 
     * 
     * GAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
     * AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
     * AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
     * HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
     * 
     * 
     * 
     * 
     * 
     * DONT YOU DARE TRY USING IT FOR ANYTHING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     * 
     * 
     * 
     * 
     * 
     * HERE FOR THE SAKE OF ARCHIVING!
     */



    //position is 0-8, level is 0-2
    /*public static Command DriveScore(RobotContainer r, int position, int level, boolean isCube){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        if(isCube){
            Vector[] armPositions = {r.arm.cals.positionCubeLow, r.arm.cals.positionCubeMed, r.arm.cals.positionCubeHi};
            Vector currentArmPos = armPositions[level];//creates the position the arm should be at

            sg.addCommands(new ArmMove(r, currentArmPos)//moves the arm to the right position during the drive
                .alongWith(new MultiDimensionalMotionProfile(r, AutonPos.SCORING_OFFSETS[position])));//drives to the scoring position

            //ejects the cube for 0.5 seconds
            sg.addCommands(new InstantCommand(() -> r.gripper.setIntakeSpeed(r.gripper.cals.cubeScoreSpeed)).raceWith(new WaitCommand(0.5)));
        } else {
            Vector[] holdArmPositions = {r.arm.cals.positionConeHiHold, r.arm.cals.positionConeMedHold, r.arm.cals.positionConeMedHold};
            Vector currentHoldArmPosition = holdArmPositions[level];//creates the initial position the arm should be at

            sg.addCommands(new ArmMove(r, currentHoldArmPosition)//moves the arm to the right position during the drive
                .alongWith(new MultiDimensionalMotionProfile(r, AutonPos.SCORING_OFFSETS[position])));//drives to the scoring position

            Vector[] scoreArmPositions = {r.arm.cals.positionConeHiRelease, r.arm.cals.positionConeMedRelease, r.arm.cals.positionConeLowRelease};
            Vector currentScoreArmPosition = scoreArmPositions[level];//creates the scoring position the arm should be at

            //moves the arm down and sets the speed to negative while moving backwards for 1 second to score the cone
            sg.addCommands(new ArmMove(r, currentScoreArmPosition)
                  .andThen(new InstantCommand(() -> r.gripper.setIntakeSpeed(r.gripper.cals.coneScoreSpeed))
                .alongWith(new DriveForTime(r, new Vector(0.2, Math.PI), 1))));
        }

        return sg;
    }

    RobotContainer r;

    public Score(RobotContainer r){
        this.r = r;
        addRequirements(r.arm);
        addRequirements(r.gripper);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return false;
    }

    public static Command armUp(RobotContainer r, Level level){
        Command command = new SequentialCommandGroup();

        if(r.inputs.isCube()){
            Vector pos;
            switch(level){
                case BOTTOM:
                    pos = r.arm.cals.positionCubeLow;
                    break;
                case MIDDLE:
                    pos = r.arm.cals.positionCubeMed;
                    break;
                case TOP:
                    pos = r.arm.cals.positionCubeHi;
                    break;
                case NONE:
                    pos = r.arm.cals.positionCubeHi;
                    break;
                default:
                    pos = r.arm.cals.positionCubeHi;
                    break;
            }
            command = new ArmMove(r, pos);
        } else {
            Vector pos;
            switch(level){
                case BOTTOM:
                    pos = r.arm.cals.positionConeLowRelease;
                    break;
                case MIDDLE:
                    pos = r.arm.cals.positionConeMedHold;
                    break;
                case TOP:
                    pos = r.arm.cals.positionConeHiHold;
                    break;
                case NONE:
                    pos = r.arm.cals.positionConeHiHold;
                    break;
                default:
                    pos = r.arm.cals.positionConeHiHold;
                    break;
            }
            command = new ArmMove(r, pos);
        }

        return command;
    }

    public static Command armScore(RobotContainer r, Level level){
        Command command = new SequentialCommandGroup();

        if(r.inputs.isCube()){
            command = GatherCommand.shootIntake(r, false);
        } else {
            Vector pos;
            switch(level){
                case BOTTOM:
                    pos = r.arm.cals.positionConeLowRelease;
                    break;
                case MIDDLE:
                    pos = r.arm.cals.positionConeMedRelease;
                    break;
                case TOP:
                    pos = r.arm.cals.positionConeHiRelease;
                    break;
                case NONE:
                    pos = r.arm.cals.positionConeHiRelease;
                    break;
                default:
                    pos = r.arm.cals.positionConeHiRelease;
                    break;
            }

            command = new ArmMove(r, pos);
        }

        return command;
    }*/
}
