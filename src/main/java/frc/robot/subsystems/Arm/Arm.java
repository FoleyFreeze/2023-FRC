package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Util;
import frc.robot.util.Vector;
import frc.robot.util.Motor.Motor;

public class Arm extends SubsystemBase {
    
    RobotContainer r;
    public ArmCal cals; 

    AnalogInput armPot;

    Motor angleMotor;
    Motor stendoMotor; 
    
    Vector setPoint;
    Vector setPointTwo;
    boolean isAngleOnly = false;

    double timeOfStendoReset;
    double stendoCurrentTime;

    Vector jogOffset = new Vector(0,0);

    public Arm(RobotContainer r, ArmCal cals){
        this.r = r;
        this.cals = cals;
        if(cals.disabled) return;

        angleMotor = Motor.create(cals.angleMotor);
        stendoMotor = Motor.create(cals.lengthMotor);
        armPot = new AnalogInput(cals.armPotChannel);
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

    public void learnArmOffset(){
        //get the arm position from a pot
        double currentAngle = armPot.getVoltage() * cals.armPotSlope + cals.armPotOffset;
        angleMotor.setEncoderPosition(currentAngle);

        //the stendo should reset to a mid-ish position, 
        //but then on retraction it should hit the stop and relearn
        stendoMotor.setEncoderPosition(cals.initialStendoPosition);
    }
    
    @Override
    public void periodic(){
        if(cals.disabled) return;

        if(setPoint != null){
            //offset and wanted setpoint combined
            setPointTwo = Vector.addVectors(setPoint, jogOffset);
            
            double currentAngle = angleMotor.getPosition();
            double setpointAngle = Math.toDegrees(setPointTwo.theta);
            double currentAngleError = setpointAngle - currentAngle;

            //interpolate - this prevents the gripper from hitting the ground
            double lengthMax = Util.interp(cals.angleAxis, cals.lengthMax, currentAngle);
            
            //interp limits again, this keeps the rotational inertia of the arm down
            double lengthMax2 = Util.interp(cals.armAngleErrorAxis, cals.stendoLengthMax, Math.abs(currentAngleError));
            lengthMax = Math.min(lengthMax,lengthMax2);

            //only letting stendo and angle move to their min/max
            double angleSetpoint = Util.bound(setPointTwo.theta, cals.angleMin, cals.angleMax);
            double lengthSetpoint = Util.bound(setPointTwo.r, cals.lengthMin, lengthMax);

            //stendo power to none and pulls arm into new position
            angleMotor.setPosition(angleSetpoint);
            if(isAngleOnly){
                stendoMotor.setPower(0);
                stendoCurrentTime = 0;
            } else {
                stendoMotor.setPosition(lengthSetpoint);
                determineStendoReset(lengthSetpoint);
            }
        }

    }

    //mathify for error
    public Vector getError(){;
        Vector currentVector = new Vector(stendoMotor.getPosition(),angleMotor.getPosition());
        return Vector.subVectors(setPointTwo, currentVector);
    }


    private void determineStendoReset(double lengthSetpoint){
        double now = Timer.getFPGATimestamp();
        if(lengthSetpoint == cals.lengthMin 
                && now - timeOfStendoReset > cals.minStendoResetTime){
            
            if(stendoMotor.getCurrent() > cals.stendoResetCurrent){
                stendoCurrentTime += r.sensors.dt;

                if(stendoCurrentTime > cals.stendoResetCurrentTime){
                    stendoMotor.setEncoderPosition(cals.lengthMin);
                    timeOfStendoReset = Timer.getFPGATimestamp();
                }
            } else {
                stendoCurrentTime = 0;
            }
            
        }
    }
}
