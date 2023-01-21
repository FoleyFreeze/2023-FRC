package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.FileManager;
import frc.robot.util.Vector;

public class DriveTrain extends SubsystemBase {

    public final boolean disabled = false;

    RobotContainer r;
    public DriveCal cals;

    public Wheel[] wheels;

    FileManager fm = new FileManager("/home/lvuser/WheelEncoderOffsets.txt");

    /* For driving, our 0 degrees point is facing forward
     * Clockwise turning is assumed to be a positive rotPwr input
     */

    public DriveTrain(RobotContainer r, DriveCal cals){
        if(disabled) return;
        this.r = r;
        this.cals = cals;

        wheels = new Wheel[cals.wheelCals.length];
        for(int i = 0; i < cals.wheelCals.length; i++){
            wheels[i] = new Wheel(cals.wheelCals[i]);
        }

        readAbsOffset();
    }

    /* Takes in x and y power values and a z power, 
     * which is a rotation pwr between -1 and 1
     */
    public void driveSwerve(Vector xy, double z){
        if(disabled) return;

        if(r.inputs.getFieldOrient()){
            xy.theta -= r.sensors.getNavXAng();
        }

        double maxWheelDist = 0;
        for(Wheel w: wheels){
            maxWheelDist = Math.max(maxWheelDist, w.wheelLocation.r);
        }

        double max = 0;
        for(Wheel w: wheels){
            //Finding the perpendicular angle from the wheel location points
            double rotAng = w.wheelLocation.theta + Math.PI/2;

            //normalize rotation vector's power based on distance from center of rot
            double rotPwr = (w.wheelLocation.r / maxWheelDist) * z;

            //Formulating vector for rotating around the center of rotation
            Vector rotVec = new Vector(rotPwr, rotAng);

            //Combining drive and rotate vectors
            w.driveVec = Vector.addVectors(xy, rotVec);

            if(w.driveVec.r > max){
                max = w.driveVec.r;
            }
        }

        normalize(max);

        for(Wheel w: wheels){
            w.drive();
        }
    }

    //Reads the wheel positions file, using our file manager logic from last year
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

    //Writes to the wheel positions file, using our file manager logic from last year
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

    //divide all numbers by the maximum so we stay at a max of 1.0 power applied to wheels
    void normalize(double max){
        if(disabled) return;
        for(Wheel w: wheels){
            if(max != 0 && Math.abs(max) > 1.0){
                w.driveVec.r /= Math.abs(max);
            }
        }
    }

    //turn all wheels inward and don't move - I imagine we will use this in lieu of driveswerve in commands
    public void parkMode(){
        for(Wheel w: wheels){
            w.driveVec.theta = w.cal.wheelLocation.theta;
            w.driveVec.r = 0;
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

    public Vector getPosition(){
        return null; //TODO: finish
    }

    public void periodic(){
        if(disabled) return;

        for(Wheel w : wheels){
            //SmartDashboard.putNumber(w.cal.idx + " enc v", w.absEncoder.getVoltage());
        }
    }
}
