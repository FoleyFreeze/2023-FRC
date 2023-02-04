package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonToolbox.SetStartPos;

public class AutonBuilder {

    public static Command buildAuton(RobotContainer r, int startPosChooser, boolean secondPieceChooser, int actionChooser, int pathChooser, int pieceChooser){
        SequentialCommandGroup sg = new SequentialCommandGroup();

        sg.addCommands(new SetStartPos(r, AutonPos.START_POSITIONS[startPosChooser]));

        return sg;
    }
}
