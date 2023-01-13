package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class DriveTrain extends SubsystemBase {

    public final boolean disabled = false;

    RobotContainer r;
    DriveCal cals;

    Wheel[] wheels;

    /* For driving, our 0 degrees point is facing forward
     * Clockwise turning is assumed to be a positive rotPwr input
     */

    public DriveTrain(RobotContainer r, DriveCal cals){
        if(disabled) return;
        this.r = r;
        this.cals = cals;

        wheels = new Wheel[4];//There are four wheels in the drivetrain and always will be
        for(int i = 0; i <= 3; i++){
            wheels[i] = new Wheel(cals.wheelCals[i]);
        }
    }

    //divide all numbers by the maximum so we stay at a max of 1.0 power applied to wheels
    void normalize(double max){
        if(disabled) return;
        for(Wheel w: wheels){
            w.driveVec.r /= max;
        }
    }

    //turn all wheels inward and don't move - I imagine we will use this in lieu of driveswerve in commands
    public void parkMode(){
        for(Wheel w: wheels){
            w.driveVec.theta = w.cal.wheelLocation.theta;
            w.driveVec.r = 0;
        }
    }

    /* Takes in an x and y position, along with 
     * rotation power to be applied to the wheels
     */
    public void driveSwerve(double x, double y, double rotPwr){
        if(disabled) return;
        Vector xy = Vector.fromXY(x, y);

        double max = 0;
        for(Wheel w: wheels){
            double rotAng = w.wheelLocation.theta;
            //Finding the perpendicular angle from the wheel location points
            if(rotPwr > 0){
                rotAng += Math.PI/2;
            } else {
                rotAng -= Math.PI/2;
            }

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

    public void periodic(){
        if(disabled) return;
        
    }
}
