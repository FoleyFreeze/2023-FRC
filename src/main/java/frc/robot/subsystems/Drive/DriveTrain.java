package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class DriveTrain extends SubsystemBase {

    RobotContainer r;
    DriveCals cals;

    public DriveTrain(RobotContainer r, DriveCals cals){
        this.r = r;
        this.cals = cals;
    }
}
