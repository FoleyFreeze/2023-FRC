package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Util;
import frc.robot.util.Vector;
import frc.robot.util.Motor.Motor;

public class Arm extends SubsystemBase {
    
    RobotContainer r;
    public ArmCal cals; 

    Motor angleMotor;
    Motor stendoMotor; 
    
    Vector setPoint;
    boolean isAngleOnly = false;

    public Arm(RobotContainer r, ArmCal cals){
        this.r = r;
        this.cals = cals;
        if(cals.disabled) return;

        angleMotor = Motor.create(cals.angleMotor);
        stendoMotor = Motor.create(cals.lengthMotor);
    }

    //move arm and stendo to new position
    public void move(Vector position){
        setPoint = position;
        isAngleOnly = false;
    }

    //only move arm 
    public void moveArmOnly(Vector position){
        setPoint = position;
        isAngleOnly = true;
    }

    
    @Override
    public void periodic(){
        if(cals.disabled) return;

        if(setPoint != null){
            double currentAngle = angleMotor.getPosition();

            //CANT REMEBER WHAT INTERP DOES
            double lengthMax = Util.interp(cals.angleAxis, cals.lengthMax, currentAngle);

            //only letting stendo and angle move to their min/max
            double angleSetpoint = Util.bound(setPoint.theta, cals.angleMin, cals.angleMax);
            double lengthSetpoint = Util.bound(setPoint.theta, cals.lengthMin, lengthMax);

            //stendo power to none and pulls arm into new position
            angleMotor.setPosition(angleSetpoint);
            if(isAngleOnly){
                stendoMotor.setPower(0);
            } else {
                stendoMotor.setPosition(lengthSetpoint);
            }
        }
    }

    //mathy for error
    public double getError(){
        double error = Math.abs(setPoint.theta - angleMotor.getPosition());
        error +=  Math.abs(setPoint.r - stendoMotor.getPosition()*4);
        return error;
    }

}
