package frc.robot.subsystems.Arm;

import java.sql.Driver;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Angle;
import frc.robot.util.Util;
import frc.robot.util.Vector;
import frc.robot.util.Motor.Motor;

public class Arm extends SubsystemBase {
    
    RobotContainer r;
    public ArmCal cals; 

    public AnalogInput armPot;

    public Motor angleMotor;
    public Motor stendoMotor; 
    
    Vector setPoint;
    //Vector setPointTwo;
    boolean isAngleOnly = false;

    public Vector jogOffset = new Vector(0,0);

    GenericEntry maxArmTempNT = Shuffleboard.getTab("Safety").add("MaxArmTemp", 0).getEntry();
    GenericEntry maxArmTempTimeNT = Shuffleboard.getTab("Safety").add("MaxArmTempTime", 0).getEntry();

    GenericEntry maxStendoTempNT = Shuffleboard.getTab("Safety").add("MaxStendoTemp", 0).getEntry();

    GenericEntry jogUpDownNT = Shuffleboard.getTab("Comp").add("Jog Up Down", 0).getEntry();
    GenericEntry jogInOutNT = Shuffleboard.getTab("Comp").add("Jog In Out", 0).getEntry();

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
        //setPointTwo = new Vector(setPoint.r + jogOffset.r, setPoint.theta + jogOffset.theta);
        isAngleOnly = false;
    }

    //only move arm 
    public void moveArmOnly(Vector position){
        setPoint = position;
        //setPointTwo = new Vector(setPoint.r + jogOffset.r, setPoint.theta + jogOffset.theta);
        isAngleOnly = true;
    }

    public void jogUp(){
        double jogDist = r.arm.cals.jogUpDist;
        if(r.inputs.shift.getAsBoolean()){
            jogDist *= 5;
        }
        jogOffset.theta += Angle.toRad(jogDist);
    }

    public void jogDown(){
        double jogDist = r.arm.cals.jogUpDist;
        if(r.inputs.shift.getAsBoolean()){
            jogDist *= 5;
        }
        jogOffset.theta -= Angle.toRad(jogDist);
    }

    public void jogIn(){
        double jogDist = r.arm.cals.jogOutDist;
        if(r.inputs.shift.getAsBoolean()){
            jogDist *= 5;
        }
        jogOffset.r -= jogDist;
    }

    public void jogOut(){
        double jogDist = r.arm.cals.jogOutDist;
        if(r.inputs.shift.getAsBoolean()){
            jogDist *= 5;
        }
        jogOffset.r += jogDist;
    }

    public void setArmOffset(double angle, double stendo){
        angleMotor.setEncoderPosition(angle);
        stendoMotor.setEncoderPosition(stendo + getStendoPulleyOffset(angle));
        jogOffset = new Vector(0,0);
    }

    //adjust stendo length to account for the rotation of the pulley 
    //making the rope shorter when the arm is up
    public double getStendoPulleyOffset(double angle){
        double rad = Math.toRadians(angle + cals.stendoPulleyAngleOffset);
        return cals.stendoPulleyLengthOffset * Math.cos(rad);
    }

    public void learnArmOffset(){
        double currentAngle = -11.6;
        angleMotor.setEncoderPosition(currentAngle);

        stendoMotor.setEncoderPosition(cals.initialStendoPosition + getStendoPulleyOffset(currentAngle));
        System.out.println("Reset arm angle/extension");
        jogOffset = new Vector(0,0);
    }
    
    double maxArmTemp = 0;
    double maxStendoTemp = 0;
    @Override
    public void periodic(){

        if(cals.disabled) return;
        SmartDashboard.putNumber("Pot Value", armPot.getVoltage());

        determineAngleBrake();

        if(r.inputs.getFieldMode()){
            angleMotor.setPIDPwrLim(0.75);
        } else {
            angleMotor.setPIDPwrLim(0.25);
        }

        if(angleMotor.getTemp() > maxArmTemp) maxArmTemp = angleMotor.getTemp();
        if(stendoMotor.getTemp() > maxStendoTemp) maxStendoTemp = stendoMotor.getTemp();

        maxArmTempNT.setDouble(maxArmTemp);
        maxStendoTempNT.setDouble(maxStendoTemp);
        
        jogUpDownNT.setDouble(Angle.toDeg(jogOffset.theta));
        jogInOutNT.setDouble(jogOffset.r);

        double currentAngle = angleMotor.getPosition() /*- Math.toDegrees(jogOffset.theta)*/;
        double currentLength = stendoMotor.getPosition() - getStendoPulleyOffset(currentAngle) /*- jogOffset.r*/;
        Vector currPos = Vector.fromDeg(currentLength, currentAngle);
        SmartDashboard.putString("ArmPos", currPos.toStringPolar());

        //TODO: find a way to do that that can actually work.
        // This strategy breaks jog completely
        //if(DriverStation.isDisabled()) setPoint = null;

        if(setPoint != null){
            //offset and wanted setpoint combined
            //move jog to after limits
            //setPointTwo = new Vector(setPoint.r + jogOffset.r, setPoint.theta + jogOffset.theta);
            
            double setpointAngle = Math.toDegrees(setPoint.theta);
            double currentAngleError = setpointAngle - currentAngle;

            //interpolate - this prevents the gripper from hitting the ground
            double lengthMax = Util.interp(cals.angleAxis, cals.lengthMax, currentAngle);
            
            //interp limits again, this keeps the rotational inertia of the arm down
            double lengthMax2 = Util.interp(cals.armAngleErrorAxis, cals.stendoLengthMax, Math.abs(currentAngleError));
            lengthMax = Math.min(lengthMax,lengthMax2);

            //only letting stendo and angle move to their min/max
            double angleSetpoint = Util.bound(Math.toDegrees(setPoint.theta), cals.angleMin, cals.angleMax);
            double lengthSetpoint = Util.bound(setPoint.r, cals.lengthMin, lengthMax);

            //adjust for pulley offset
            lengthSetpoint += getStendoPulleyOffset(currentAngle);

            //add jog r and theta
            angleSetpoint += Math.toDegrees(jogOffset.theta);
            lengthSetpoint += jogOffset.r;

            //stendo power to none and pulls arm into new position
            angleMotor.setPosition(angleSetpoint);
            if(isAngleOnly){
                //stendoMotor.setPower(0);
                //stendoCurrentTime = 0;
            } else {
                stendoMotor.setPosition(lengthSetpoint);
                //System.out.println(stendoMotor.getPosition() + "  " + lengthSetpoint);
                determineStendoReset(currentAngle);
            }
        }
    }

    
    boolean ranAutoAtLeastOnce = false;
    private void determineAngleBrake(){
        if(!ranAutoAtLeastOnce && DriverStation.isAutonomous() && DriverStation.isEnabled()){
            ranAutoAtLeastOnce = true;
        }
        if(ranAutoAtLeastOnce && DriverStation.isFMSAttached()){
            angleMotor.setBrakeMode(true);
        } else {
            angleMotor.setBrakeMode(!DriverStation.isDisabled());
        }
    }

    //mathify for error
    public Vector getError(){;
        Vector currentVector = Vector.fromDeg(stendoMotor.getPosition() - jogOffset.r,angleMotor.getPosition() - Math.toDegrees(jogOffset.theta));
        return Vector.subVectors(setPoint, currentVector);
    }


    //if stendo is hitting the current limit for > 1s, reset the 0
    double stendoResetTime = 0;
    double stendoResetPosition = 0;
    private void determineStendoReset(double currentAngle){
        double now = Timer.getFPGATimestamp();
        double pos = stendoMotor.getPosition();
        double posErr = Math.abs(pos - stendoResetPosition);
        if(stendoMotor.getCurrent() > cals.stendoCurrLim-5 && posErr < 0.1){
            if(stendoResetTime < now-0.2) {
                stendoResetTime = now + 0.5;
            } else if(stendoResetTime < now){
                stendoMotor.setEncoderPosition(cals.lengthMin + getStendoPulleyOffset(currentAngle));
                stendoMotor.setPosition(cals.lengthMin + getStendoPulleyOffset(currentAngle));
                jogOffset.r = 0;
            }
        } else {
            stendoResetPosition = pos;
        }
    }
}
