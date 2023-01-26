package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Motor.Motor;

public class Arm extends SubsystemBase {
    
    RobotContainer r;
    ArmCal cals; 

    Motor angleMotor;
    Motor stendoMotor; 

    public Arm(RobotContainer r, ArmCal cals){
        if(cals.disabled) return;
        this.r = r;
        this.cals = cals;
    }


}
