package frc.robot.subsystems.Inputs;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tabs {

    public ShuffleboardTab safetyTab = Shuffleboard.getTab("Safety");
    public ShuffleboardTab compTab = Shuffleboard.getTab("Comp");
    public ShuffleboardTab buttonsTab = Shuffleboard.getTab("Buttons");

    public Tabs(){

    }
}
