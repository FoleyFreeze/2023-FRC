package frc.robot.subsystems.Sensors;

import frc.robot.subsystems.Drive.DriveCal;
import frc.robot.util.Vector;

public class Odometry implements AutoCloseable {
    
    OdometryCals cals;
    public DriveCal driveCals;

    public Vector botLocation;

    public Odometry(OdometryCals cals, DriveCal driveCals){
        this.cals = cals;
        this.driveCals = driveCals;
    }

    private Vector[] prevWheelStates;
    private double prevBotAng;
    public void update(double botAng, Vector[] wheelStates){
        if(prevWheelStates == null) prevWheelStates = wheelStates;
        int wheelNum = wheelStates.length;

        double deltaAng = botAng - prevBotAng;
        
        //formulate actual vectors based on angle and distance traveled
        Vector[] strafeVecs = new Vector[wheelNum];
        for(int i = 0; i < wheelNum; i++){
            //based on whether we want to take an average of the current and previous wheel angles or just use the current
            double wheelAngle;
            if(cals.averageWheelAng){
                wheelAngle = (wheelStates[i].theta - prevWheelStates[i].theta) / 2.0;
            } else {
                wheelAngle = wheelStates[i].theta;
            }
            strafeVecs[i] = new Vector(wheelStates[i].r - prevWheelStates[i].r, wheelAngle);
        }

        //back-calculate rotation vectors based on wheel locations and delta angle from navX
        Vector[] rotVecs = new Vector[wheelNum];
        for(int i = 0; i < wheelNum; i++){
            double rotAng = driveCals.wheelCals[i].wheelLocation.theta + Math.PI / 2.0;
            double r = driveCals.wheelCals[i].wheelLocation.r * deltaAng;
            rotVecs[i] = new Vector(r, rotAng);
        }

        //subtract the rotation vectors away from the originals
        Vector[] resultVecs = new Vector[wheelNum];
        for(int i = 0; i < wheelNum; i++){
            resultVecs[i] = Vector.addVectors(strafeVecs[i], rotVecs[i].negate());
        }

        Vector[] finalVecs = checkVCriteria(resultVecs);
        botLocation.add(Vector.averageVectors(finalVecs));

        prevWheelStates = wheelStates;
    }

    public void setBotLocation(Vector location){
        botLocation = location;
    }

    public Vector[] checkVCriteria(Vector[] vecs){
        Vector[] output = vecs;

        double average = 0;
        for(Vector v: output){
            average += v.r;
        }
        average = average / output.length;

        double sum = 0;
        for(Vector v: output){
            sum += (v.r - average) * (v.r - average);
        }
        double stanDeviation = Math.sqrt(sum / output.length);

        for(int i = 0; i < output.length; i++){
            if(Math.abs(average - output[i].r) > Math.abs(stanDeviation * cals.maxStandardDeviations)){
                output[i] = null;
            }
        }

        return output;
    }

    @Override
    public void close(){

    }
}
