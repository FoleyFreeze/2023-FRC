package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.util.FileManager;
import frc.robot.util.Vector;

public class DriveTrain extends SubsystemBase {

    RobotContainer r;
    public DriveCal cals;

    public Wheel[] wheels;

    FileManager fm = new FileManager("/home/lvuser/WheelEncoderOffsets.txt");

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

        readAbsOffset();
    }

    /* Takes in x and y power values and a z power, 
     * which is a rotation pwr between -1 and 1
     */
    public void driveSwerve(Vector xy, double z){
        if(cals.disabled) return;

        //Grabs the calculated drive vectors and sets it into the actual wheel array
        Vector[] wheelLocations = new Vector[wheels.length];
        for(int i = 0; i < wheels.length; i++){
            wheelLocations[i] = wheels[i].wheelLocation;
        }

        Vector[] driveVecs = formulateDriveVecs(xy, z, wheels.length, wheelLocations);
        for(int i = 0; i < wheels.length; i++){
            wheels[i].driveVec = driveVecs[i];
        }

        for(Wheel w: wheels){
            w.drive();
        }
    }

    //This guy separates math from setting values to actual wheels
    public static Vector[] formulateDriveVecs(Vector xy, double z, int wheelLength, Vector[] wheelLocations)
    {
        Vector[] driveVecs = new Vector[wheelLength];

        double maxWheelDist = 0;
        for(int i = 0; i < wheelLength; i++){
            maxWheelDist = Math.max(maxWheelDist, wheelLocations[i].r);
        }

        double max = 0;
        for(int i = 0; i < wheelLength; i++){
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
        }

        //divide all numbers by the maximum so we stay at a max of 1.0 power applied to wheels
        if(Math.abs(max) > 1.0){
            for(int i = 0; i < driveVecs.length; i++){
                driveVecs[i].r /= Math.abs(max);
            }
        }

        return driveVecs;
    }

    public void swerveMP(Vector velocity, double accel){
        //SmartDashboard.putNumber("velocity", velocity.r);
        Vector Power = new Vector(velocity);
        Power.r = (AutonCal.kA * accel) + (AutonCal.kV * velocity.r) + AutonCal.kS;
        driveSwerve(Power, 0);
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
        for(int i = 0; i < wheels.length; i++){
            wheels[i].driveVec.theta = wheels[i].cal.wheelLocation.theta;
            wheels[i].driveVec.r = 0;
        }
    }

    //get all four wheel vectors
    public Vector[] getWheelState(){
        Vector[] wheelVectors = new Vector[cals.wheelCals.length];
        for(int i = 0; i < wheelVectors.length; i++){
            wheelVectors[i] = new Vector(wheels[i].getDist(), wheels[i].getAng());
        }
        return wheelVectors;
    }

    public void periodic(){
        if(cals.disabled) return;

        for(Wheel w : wheels){
            SmartDashboard.putNumber("WheelTemp " + w.idx, w.swerveMotor.getTemp());
        }
    }
}
