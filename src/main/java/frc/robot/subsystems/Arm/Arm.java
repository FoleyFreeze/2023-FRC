package frc.robot.subsystems.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    GenericEntry maxArmTempNT = Shuffleboard.getTab("Safety").add("MaxArmTemp", 0).getEntry();
    GenericEntry maxArmTempTimeNT = Shuffleboard.getTab("Safety").add("MaxArmTempTime", 0).getEntry();

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
        setPointTwo = Vector.addVectors(setPoint, jogOffset);
        isAngleOnly = false;
    }

    //only move arm 
    public void moveArmOnly(Vector position){
        setPoint = position;
        setPointTwo = Vector.addVectors(setPoint, jogOffset);
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

    public void setArmOffset(double angle, double stendo){
        angleMotor.setEncoderPosition(angle);
        stendoMotor.setEncoderPosition(stendo + getStendoPulleyOffset(angle));
    }

    public void learnArmOffset(){
        //get the arm position from a pot
        //TODO: add this back in when this pot exists
        //double currentAngle = armPot.getVoltage() * cals.armPotSlope + cals.armPotOffset;
        double currentAngle = 0;
        angleMotor.setEncoderPosition(currentAngle);

        //the stendo should reset to a mid-ish position, 
        //but then on retraction it should hit the stop and relearn
        stendoMotor.setEncoderPosition(cals.initialStendoPosition + getStendoPulleyOffset(currentAngle));
    }

    //adjust stendo length to account for the rotation of the pulley 
    //making the rope shorter when the arm is up
    private double getStendoPulleyOffset(double angle){
        double rad = Math.toRadians(angle + cals.stendoPulleyAngleOffset);
        return cals.stendoPulleyLengthOffset * Math.cos(rad);
    }
    
    double maxArmCurr = 0;
    double maxArmTemp = 0;
    double maxArmTempTime = 0;
    @Override
    public void periodic(){

        if(cals.disabled) return;
        angleMotor.setBrakeMode(!DriverStation.isDisabled());

        SmartDashboard.putNumber("ArmAngleTemp",angleMotor.getTemp());
        SmartDashboard.putNumber("ArmStendoTemp",stendoMotor.getTemp());
        //SmartDashboard.putNumber("ArmCurrent",angleMotor.getCurrent());
        if(maxArmCurr < angleMotor.getCurrent()) maxArmCurr = angleMotor.getCurrent();
        SmartDashboard.putNumber("MaxArmCurrent", maxArmCurr);
        if(maxArmTemp < angleMotor.getTemp()) {
            maxArmTemp = angleMotor.getTemp();
            maxArmTempTime = Timer.getFPGATimestamp();
        }
        maxArmTempNT.setDouble(maxArmTemp);
        maxArmTempTimeNT.setDouble(maxArmTempTime);
        

        double currentAngle = angleMotor.getPosition();
        double currentLength = stendoMotor.getPosition() - getStendoPulleyOffset(currentAngle);
        Vector currPos = Vector.fromDeg(currentLength, currentAngle);
        SmartDashboard.putString("ArmPos", currPos.toStringPolar());

        if(DriverStation.isDisabled()) setPoint = null;

        if(setPoint != null){
            //offset and wanted setpoint combined
            setPointTwo = Vector.addVectors(setPoint, jogOffset);
            
            double setpointAngle = Math.toDegrees(setPointTwo.theta);
            double currentAngleError = setpointAngle - currentAngle;

            //interpolate - this prevents the gripper from hitting the ground
            double lengthMax = Util.interp(cals.angleAxis, cals.lengthMax, currentAngle);
            
            //interp limits again, this keeps the rotational inertia of the arm down
            double lengthMax2 = Util.interp(cals.armAngleErrorAxis, cals.stendoLengthMax, Math.abs(currentAngleError));
            lengthMax = Math.min(lengthMax,lengthMax2);

            //only letting stendo and angle move to their min/max
            double angleSetpoint = Util.bound(Math.toDegrees(setPointTwo.theta), cals.angleMin, cals.angleMax);
            double lengthSetpoint = Util.bound(setPointTwo.r, cals.lengthMin, lengthMax);

            //adjust for pulley offset
            lengthSetpoint += getStendoPulleyOffset(currentAngle);

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
        Vector currentVector = Vector.fromDeg(stendoMotor.getPosition(),angleMotor.getPosition());
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
