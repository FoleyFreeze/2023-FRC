package frc.robot.subsystems.Arm;

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

    AnalogInput armPot;

    public Motor angleMotor;
    public Motor stendoMotor; 
    
    Vector setPoint;
    //Vector setPointTwo;
    boolean isAngleOnly = false;

    double timeOfStendoReset;
    double stendoCurrentTime;

    Vector jogOffset = new Vector(0,0);

    GenericEntry maxArmTempNT = Shuffleboard.getTab("Safety").add("MaxArmTemp", 0).getEntry();
    GenericEntry maxArmTempTimeNT = Shuffleboard.getTab("Safety").add("MaxArmTempTime", 0).getEntry();

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

    public void learnArmOffset(){
        //get the arm position from a pot
        //TODO: add this back in when this pot exists
        //double currentAngle = armPot.getVoltage() * cals.armPotSlope + cals.armPotOffset;
        double currentAngle = -11;
        angleMotor.setEncoderPosition(currentAngle);

        //the stendo should reset to a mid-ish position, 
        //but then on retraction it should hit the stop and relearn
        stendoMotor.setEncoderPosition(cals.initialStendoPosition + getStendoPulleyOffset(currentAngle));
        System.out.println("Reset arm angle/extension");
        jogOffset = new Vector(0,0);
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
        SmartDashboard.putNumber("Pot Value", armPot.getVoltage());

        angleMotor.setBrakeMode(!DriverStation.isDisabled());
        if(r.inputs.getFieldMode()){
            angleMotor.setPIDPwrLim(0.75);
        } else {
            angleMotor.setPIDPwrLim(0.25);
        }

        SmartDashboard.putNumber("ArmAngleTemp",angleMotor.getTemp());
        SmartDashboard.putNumber("ArmStendoTemp",stendoMotor.getTemp());
        double armCurrent = angleMotor.getCurrent();
        SmartDashboard.putNumber("ArmCurrent",armCurrent);
        if(maxArmCurr < armCurrent) maxArmCurr = angleMotor.getCurrent();
        SmartDashboard.putNumber("MaxArmCurrent", maxArmCurr);
        if(maxArmTemp < angleMotor.getTemp()) {
            maxArmTemp = angleMotor.getTemp();
            maxArmTempTime = Timer.getFPGATimestamp();
        }
        maxArmTempNT.setDouble(maxArmTemp);
        maxArmTempTimeNT.setDouble(maxArmTempTime);
        
        jogUpDownNT.setDouble(Angle.toDeg(jogOffset.theta));
        jogInOutNT.setDouble(jogOffset.r);

        double currentAngle = angleMotor.getPosition() - Math.toDegrees(jogOffset.theta);
        double currentLength = stendoMotor.getPosition() - getStendoPulleyOffset(currentAngle) - jogOffset.r;
        Vector currPos = Vector.fromDeg(currentLength, currentAngle);
        SmartDashboard.putString("ArmPos", currPos.toStringPolar());

        if(DriverStation.isDisabled()) setPoint = null;

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
            SmartDashboard.putNumber("Arm Setpoint", angleSetpoint);
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
        Vector currentVector = Vector.fromDeg(stendoMotor.getPosition() - jogOffset.r,angleMotor.getPosition() - Math.toDegrees(jogOffset.theta));
        return Vector.subVectors(setPoint, currentVector);
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
