package frc.robot.subsystems.Drive;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.subsystems.Inputs.Inputs.Level;
import frc.robot.subsystems.Inputs.Inputs.ManScoreMode;
import frc.robot.util.Angle;
import frc.robot.util.FileManager;
import frc.robot.util.Util;
import frc.robot.util.Vector;

public class DriveTrain extends SubsystemBase {

    RobotContainer r;
    public DriveCal cals;

    public Wheel[] wheels;

    FileManager fm = new FileManager("/home/lvuser/WheelEncoderOffsets.txt");

    GenericEntry driveTempNT;
    GenericEntry swerveTempNT;

    /* For driving, our 0 degrees point is facing forward
     * Clockwise turning is assumed to be a positive rotPwr input
     */

    public DriveTrain(RobotContainer r, DriveCal cals){
        this.r = r;
        this.cals = cals;
        if(cals.disabled) return;

        wheels = new Wheel[cals.wheelCals.length];
        for(int i = 0; i < cals.wheelCals.length; i++){
            wheels[i] = new Wheel(cals.wheelCals[i]);
            wheels[i].resetPosition(0);
        }

        targetHeading = r.sensors.getNavXAng();

        readAbsOffset();

        driveTempNT = Shuffleboard.getTab("Safety").add("driveTemps", "0, 0, 0, 0").withSize(2, 1).withPosition(0, 2).getEntry();
        swerveTempNT = Shuffleboard.getTab("Safety").add("swerveTemps", "0, 0, 0, 0").withSize(2, 1).withPosition(0, 3).getEntry();
    }

    public boolean parkMode = false;
    public void setParkMode(boolean parkMode){
        this.parkMode = parkMode;
        System.out.println("Park mode set " + parkMode);
    }

    //automatically maintains heading if rotation is not commanded
    public enum HeadingSource{
        Angle,Shelf,Score,Auto,Drive,Broke
    }
    HeadingSource hs = HeadingSource.Angle;
    public double lastRotateTime = 0;
    double iAccum = 0;
    public double targetHeading;
    public double swerveAutoAngle(double drivePower, double anglePower){
        //auto align, only if the navx exists
        double z = anglePower;
        if(r.sensors.navX.isConnected() /*r.inputs.getFieldAlign()*/){
            if(anglePower != 0){
                lastRotateTime = Timer.getFPGATimestamp() + cals.autoAngleWaitTime;
                iAccum = 0;
                targetHeading = r.sensors.odo.botAngle;
                hs = HeadingSource.Angle;
            } else if(DriverStation.isAutonomousEnabled() || Timer.getFPGATimestamp() > lastRotateTime) {
                //select setpoint
                double setpoint = targetHeading;
                boolean usePID = drivePower > 0 || (cals.autoAngleDuringCoast && r.sensors.odo.deltaBotLocation.r > cals.autoCoastAngleMinVel);
                if(r.inputs.cameraMode() && !(r.inputs.autoGather.getAsBoolean() && r.inputs.disableAutoGather) && !DriverStation.isAutonomous() && r.inputs.getFieldMode()){
                    usePID |= r.inputs.autoGather.getAsBoolean() || r.inputs.autoScore.getAsBoolean();
                } else if(r.inputs.autoGather.getAsBoolean() && r.inputs.isShelf() && !DriverStation.isAutonomous()){
                    //target shelf angle
                    setpoint = Math.toRadians(0);
                    targetHeading = r.sensors.odo.botAngle;
                    hs = HeadingSource.Shelf;
                    usePID = true;
                } else if(r.inputs.scoreMode == ManScoreMode.SCORE && r.inputs.selectedLevel == Level.TOP && !r.inputs.isCube() && !DriverStation.isAutonomous()){
                    //target score angle for lvl3 cones
                    usePID = true;
                    setpoint = Math.toRadians(180);
                    /*if(r.inputs.fieldAlignRight.getAsBoolean()){
                        setpoint = Math.toRadians(-164.25);
                    } else {
                        setpoint = Math.toRadians(-195);
                    }*/
                    targetHeading = r.sensors.odo.botAngle;
                    hs = HeadingSource.Score;
                } else if(r.inputs.scoreMode == ManScoreMode.SCORE && (r.inputs.selectedLevel != Level.TOP || r.inputs.isCube()) && !DriverStation.isAutonomous()){
                    usePID = true;
                    setpoint = Math.toRadians(180);
                    targetHeading = r.sensors.odo.botAngle;
                    hs = HeadingSource.Score;
                }
                if(!r.inputs.getFieldMode()) usePID = false;
                SmartDashboard.putBoolean("UseAnglePID", usePID);

                //error is between -360 -> 360
                double error = (setpoint - r.sensors.odo.botAngle) % (2*Math.PI);
                if(Math.abs(error) > Math.PI){
                    if(error > 0) error -= 2*Math.PI;
                    else error += 2*Math.PI;
                }
                
                if(!usePID){
                    z = 0;
                    iAccum = 0;
                } else if(Math.abs(error) > Math.toRadians(8)){
                    //keep i low to prevent oscillation
                    z = error * r.driveTrain.cals.autoAlignKp + iAccum * r.driveTrain.cals.autoAlignKi + r.sensors.odo.deltaBotAngle * r.driveTrain.cals.autoAlignKd;
                    z = Util.bound(z, -r.driveTrain.cals.autoAlignMaxPower, r.driveTrain.cals.autoAlignMaxPower);
                    iAccum = 0;
                } else {
                    iAccum += error * r.sensors.dt;
                    z = error * r.driveTrain.cals.autoAlignKp + iAccum * r.driveTrain.cals.autoAlignKi + r.sensors.odo.deltaBotAngle * r.driveTrain.cals.autoAlignKd;
                    z = Util.bound(z, -r.driveTrain.cals.autoAlignMaxPower, r.driveTrain.cals.autoAlignMaxPower);
                }
                //System.out.format("Src: %s, Ang: %.0f, e: %.1f, i: %.1f, d: %.0f\n",hs.name(),Math.toDegrees(r.sensors.odo.botAngle),Math.toDegrees(error),Math.toDegrees(iAccum),Math.toDegrees(r.sensors.odo.deltaBotAngle));
            } else {
                iAccum = 0;
                targetHeading = r.sensors.odo.botAngle;
                hs = HeadingSource.Angle;
            }

        } else {
            //no navx
            iAccum = 0;
            targetHeading = r.sensors.odo.botAngle;
            hs = HeadingSource.Broke;
            lastRotateTime = Timer.getFPGATimestamp() + cals.autoAngleWaitTime;
        }

        return z;
    }

    public void setLastRotateTime(double time){
        lastRotateTime = time;
    }

    //same as drive swerve but hold this angle
    public void driveSwerveAngle(Vector xy, double angle){
        targetHeading = angle;
        hs = HeadingSource.Drive;
        setLastRotateTime(0);
        driveSwerve(xy, 0);
    }

    /* Takes in x and y power values and a z power, 
     * which is a rotation pwr between -1 and 1
     */
    public void driveSwerve(Vector xy, double z){
        if(cals.disabled) return;

        //maintain selected heading
        double newZ = swerveAutoAngle(xy.r, z);
        boolean coasting = cals.autoAngleDuringCoast && z == 0 && xy.r == 0 && Math.abs(newZ) > 0;
        if(coasting){
            //if we are trying to maintain an angle while coasting
            //create a fake drive vector so that the robot can still coast
            //and then only change wheel angles so that it coasts in the correct direction
            Vector vel = new Vector(r.sensors.odo.deltaBotLocation);
            vel.r /= 120.0;
            vel.r = Util.bound(vel.r, 0.1, 0.8);
            //TODO: unsure if targetHeading or botAngle is better for the field oriented offset here 
            xy = new Vector(vel.r, vel.theta - r.sensors.odo.botAngle);   
        }
        z = newZ;

        //Grabs the calculated drive vectors and sets it into the actual wheel array
        Vector[] wheelLocations = new Vector[wheels.length];
        for(int i = 0; i < wheels.length; i++){
            wheelLocations[i] = wheels[i].wheelLocation;
        }

        Vector[] driveVecs = formulateDriveVecs(xy, z, wheels.length, wheelLocations, parkMode);
        for(int i = 0; i < wheels.length; i++){
            wheels[i].driveVec = driveVecs[i];
        }

        for(Wheel w: wheels){
            w.drive(parkMode,coasting);
            if(parkMode){
                w.driveMotor.setBrakeMode(true);
            } else {
                w.driveMotor.setBrakeMode(false);
            }
        }
    }

    public void driveAngleOnly(double angle){
        for(Wheel w : wheels){
            w.driveAngleOnly(angle);
        }
    }

    //This guy separates math from setting values to actual wheels
    public static Vector[] formulateDriveVecs(Vector xy, double z, int wheelLength, Vector[] wheelLocations, boolean parkMode){
        Vector[] driveVecs = new Vector[wheelLength];

        double maxWheelDist = 0;
        for(int i = 0; i < wheelLength; i++){
            maxWheelDist = Math.max(maxWheelDist, wheelLocations[i].r);
        }

        double max = 0;
        for(int i = 0; i < wheelLength; i++){
            if(!parkMode){
                //Finding the perpendicular angle from the wheel location points
                double rotAng = wheelLocations[i].theta + Math.PI/2;

                //normalize rotation vector's power based on distance from center of rot
                double rotPwr = (wheelLocations[i].r / maxWheelDist) * z;

                //Formulating vector for rotating around the center of rotation
                Vector rotVec = new Vector(rotPwr, rotAng);

                //Combining drive and rotate vectors
                driveVecs[i] = Vector.addVectors(xy, rotVec);

                if(Math.abs(driveVecs[i].r) > max){
                    max = Math.abs(driveVecs[i].r);
                }
            } else {
                //Finding the perpendicular angle from the wheel location points
                double rotAng = wheelLocations[i].theta;

                //Combining drive and rotate vectors
                driveVecs[i] = new Vector(0, rotAng);

                if(Math.abs(driveVecs[i].r) > max){
                    max = Math.abs(driveVecs[i].r);
                }
            }
        }

        //divide all numbers by the maximum so we stay at a max of 1.0 power applied to wheels
        if(Math.abs(max) > 1.0){
            for(int i = 0; i < driveVecs.length; i++){
                driveVecs[i].r /= Math.abs(max);
            }
        }

        return driveVecs;
    }

    public void swerveMP(Vector power, double targetAngle){
        targetHeading = targetAngle;
        hs = HeadingSource.Auto;
        power.theta -= r.sensors.odo.botAngle;
        driveSwerve(power, 0);
    }

    public void swerveMPAng(Vector strafePwr, double angPwr){
        strafePwr.theta -= r.sensors.odo.botAngle;
        driveSwerve(strafePwr, angPwr);
    }

    public void swerveMPA(double power){
        driveSwerve(new Vector(0,0), power);
    }

    //Reads the wheel positions file, using our file manager logic from 2022
    public void readAbsOffset(){
        try{
            if(fm.exists()){
                System.out.println("Reading wheel positions file:");
                for(Wheel w : wheels) {
                    double val = Double.parseDouble(fm.readLine());
                    System.out.println(w.cal.name + " " + val);
                    w.setEncAngOffset(val);
                }

                fm.close();
                }
        }catch(Exception e){
            System.out.println(e.toString());
            e.printStackTrace();
            //if there was an error, reset to cal value
            System.out.println("Error reading file, defaulting to:");
            for(Wheel w : wheels) {
                w.setEncAngOffset(0);
                System.out.println(w.cal.name + " " + 0);
            }
        }
    }

    //Writes to the wheel positions file, using our file manager logic from 2022
    public void writeAbsOffset(){
        try{
            System.out.println("Saving new wheel locations:");
            for(Wheel w: wheels){
                double voltage = w.absEncoder.getVoltage();
                fm.writeLine(Double.toString(voltage));
                w.setEncAngOffset(voltage);
                System.out.println(w.cal.name + " " + voltage);
            }
            fm.close();
        }catch(Exception e){
            System.out.println("Error while saving wheel locations:");
            System.out.println(e.toString());
            e.printStackTrace();
        }
    }

    //turn all wheels inward and don't move - I imagine we will use this in lieu of driveswerve in commands
    public void parkMode(){
        
    }

    //get all four wheel vectors
    public Vector[] getWheelState(){
        Vector[] wheelVectors = new Vector[cals.wheelCals.length];
        for(int i = 0; i < wheelVectors.length; i++){
            wheelVectors[i] = new Vector(wheels[i].getDist(), wheels[i].getAng());
        }
        return wheelVectors;
    }

    boolean prevFieldTele = false;
    public void periodic(){
        if(cals.disabled) return;

        SmartDashboard.putNumber("RRDrivePosition", wheels[3].driveMotor.getPosition());

        if(prevFieldTele && DriverStation.isDisabled()){
            for(Wheel w : wheels){
                w.driveMotor.setBrakeMode(true);
            }
        }
        prevFieldTele = DriverStation.isFMSAttached() && DriverStation.isTeleopEnabled();
        

        String driveTemps = "";
        String swerveTemps = "";
        for(int i = 0; i < 4; i++){
            driveTemps += wheels[i].driveMotor.getTemp() + ", ";
            swerveTemps += wheels[i].swerveMotor.getTemp() + ", ";
        }

        driveTempNT.setString(driveTemps);
        swerveTempNT.setString(swerveTemps);

        double currentTotal = 0;
        for(Wheel w : wheels){
            currentTotal += w.driveMotor.getCurrent();
            SmartDashboard.putNumber(w.cal.name + " Abs", w.absEncoder.getVoltage());
        }
        //SmartDashboard.putNumber("Drive Current",currentTotal);
        
    }

    public void resetWheelReads(){
        for(Wheel w : wheels){
            w.resetPosReads();
        }
    }
}
