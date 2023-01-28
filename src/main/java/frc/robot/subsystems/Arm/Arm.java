package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Util;
import frc.robot.util.Vector;
import frc.robot.util.Motor.Motor;

public class Arm extends SubsystemBase {
    
    RobotContainer r;
    ArmCal cals; 

    Motor angleMotor;
    Motor stendoMotor; 
    
    Vector setPoint;

    public Arm(RobotContainer r, ArmCal cals){
        if(cals.disabled) return;
        this.r = r;
        this.cals = cals;

        angleMotor = Motor.create(cals.angleMotor);
        stendoMotor = Motor.create(cals.lengthMotor);
    }

    public void move(Vector position){
        setPoint = position;
    }

    @Override
    public void periodic(){
        if(cals.disabled) return;

        double currentAngle = angleMotor.getPosition();

        double lengthMax = Util.interp(cals.angleAxis, cals.lengthMax, currentAngle);

        double angleSetpoint = Util.bound(setPoint.theta, cals.angleMin, cals.angleMax);
        double lengthSetpoint = Util.bound(setPoint.theta, cals.lengthMin, lengthMax);

        angleMotor.setPosition(angleSetpoint);
        stendoMotor.setPosition(lengthSetpoint);
    }

}
