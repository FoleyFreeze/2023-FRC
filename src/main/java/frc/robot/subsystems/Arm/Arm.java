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
    Vector setPointTwo;
    boolean isAngleOnly = false;

    Vector jogOffset = new Vector(0,0);

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

    public void jogUp(){
        jogOffset.incrmntX(-r.arm.cals.jogUpDist);
    }

    public void jogDown(){
        jogOffset.incrmntX(r.arm.cals.jogUpDist);
    }

    public void jogIn(){
        jogOffset.incrmntY(-r.arm.cals.jogOutDist);
    }

    public void jogOut(){
        jogOffset.incrmntY(r.arm.cals.jogOutDist);
    }
    
    @Override
    public void periodic(){
        if(cals.disabled) return;

        if(setPoint != null){
            double currentAngle = angleMotor.getPosition();

            //offset and wanted setpoint combined
            setPointTwo = Vector.addVectors(setPoint, jogOffset);

            //interpolate - this prevents the gripper from hitting the ground
            double lengthMax = Util.interp(cals.angleAxis, cals.lengthMax, currentAngle);
            

            //only letting stendo and angle move to their min/max
            double angleSetpoint = Util.bound(setPointTwo.theta, cals.angleMin, cals.angleMax);
            double lengthSetpoint = Util.bound(setPointTwo.r, cals.lengthMin, lengthMax);

            //stendo power to none and pulls arm into new position
            angleMotor.setPosition(angleSetpoint);
            if(isAngleOnly){
                stendoMotor.setPower(0);
            } else {
                stendoMotor.setPosition(lengthSetpoint);
            }
        }
    }

    //mathify for error
    public double getError(){
        double error = Math.abs(setPointTwo.theta - angleMotor.getPosition());
        error +=  Math.abs(setPointTwo.r - stendoMotor.getPosition()*4);
        return error;
    }

}
