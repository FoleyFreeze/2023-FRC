package frc.robot.subsystems.Sensors;

import java.nio.charset.StandardCharsets;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive.DriveCal;
import frc.robot.subsystems.Drive.DriveCal.WheelCal;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class Odometry implements AutoCloseable {
    
    OdometryCals cals;
    public WheelCal[] wCal;

    public Vector botLocation;
    public double botAngle;//TODO: use this in place of navX angle in all places it matters

    public Odometry(OdometryCals cals, WheelCal[] wCal){
        this.cals = cals;
        this.wCal = wCal;

        setBotLocation(Vector.fromXY(0, 0));
    }

    public void setBotLocation(Vector location){
        botLocation = location;
    }

    public int badWheels = 0;
    private Vector[] prevWheelStates;
    private double prevBotAng;
    public void update(double botAng, Vector[] wheelStates){
        if(prevWheelStates == null) {
            prevWheelStates = wheelStates;
            prevBotAng = botAng;
        }
        int wheelNum = wheelStates.length;

        //formulate actual vectors based on angle and distance traveled
        Vector[] realVecs = new Vector[wheelNum];
        for(int i = 0; i < wheelNum; i++){
            //based on whether we want to take an average of the current and previous wheel angles or just use the current
            double wheelAngle;
            if(cals.averageWheelAng){
                wheelAngle = ((wheelStates[i].theta + botAng) + (prevWheelStates[i].theta + prevBotAng)) / 2.0;
            } else {
                wheelAngle = wheelStates[i].theta + botAng;
            }
            realVecs[i] = new Vector(wheelStates[i].r - prevWheelStates[i].r, wheelAngle);
        }

        getWheelDiffsOdo(botAng, realVecs);
        //getStdDevOdo(botAng, realVecs);
        
        prevWheelStates = wheelStates;
        prevBotAng = botAng;
    }

    //all possible wheel combo groups (as indexes)
    int[][] groups = {{0,1},
                      {1,2},
                      {2,3},
                      {3,0},
                      {0,2},
                      {1,3},
                      {0,1,2},
                      {1,2,3},
                      {2,3,0},
                      {3,0,1},
                      {0,1,2,3}
                    };
    
    public double totalError = 0;
    public void getWheelDiffsOdo(double botAng, Vector[] realVecs){
        
        double minError = Double.POSITIVE_INFINITY;//Yuh
        double error = 0;
        Vector bestStrafe = new Vector(0, 0);
        double bestAngle = 0;
        for(int[] group : groups){

            Vector strafeAverage = Vector.averageSomeVectors(realVecs, group);//Get strafe value from averaging every wheel vector in the group

            double totalRotError = 0;//Total group of error values based on residual vs. actual rotation
            double rotationAngles = 0;//Total group of angle arcs combined (will be averaged later)
            for(int i : group){
                Vector angFromWheels = Vector.addVectors(realVecs[i], strafeAverage.negate());//subtract the strafe from the real wheel angles to get rotation vectors
                double angError = Math.abs(angFromWheels.theta - (wCal[i].wheelLocation.theta + Math.PI / 2));
                totalRotError += Math.sin(angError) * angFromWheels.r;

                rotationAngles += angFromWheels.r / wCal[i].wheelLocation.r;
            }

            double avgRotationAngles = rotationAngles / group.length;
            double avgRotAngError = totalRotError / group.length;

            //Save all values of the group based on how good we think it is
            if(avgRotAngError < minError){
                minError = avgRotAngError;
                bestStrafe = strafeAverage;
                bestAngle = avgRotationAngles;
            }

            error = totalRotError;//Save this to the outer scope
        }
        
        //Send out a total error for a +- value
        totalError += error;
        SmartDashboard.putNumber("Rotation Error", error);

        //Set the global values
        bestStrafe.theta += botAngle;//Convert to field relative
        botAngle += bestAngle;
        botLocation.add(bestStrafe);
    }

    public void getStdDevOdo(double botAng, Vector[] realVecs){
        int wheelNum = 4;
        double deltaAng = botAng - prevBotAng;

        //back-calculate rotation vectors based on wheel locations and delta angle from navX
        Vector[] rotVecs = new Vector[wheelNum];
        for(int i = 0; i < wheelNum; i++){
            double rotAng = wCal[i].wheelLocation.theta + Math.PI / 2.0;
            double r = wCal[i].wheelLocation.r * deltaAng;
            rotVecs[i] = new Vector(r, rotAng);
        }

        //subtract the rotation vectors away from the originals
        Vector[] resultVecs = new Vector[wheelNum];
        for(int i = 0; i < wheelNum; i++){
            resultVecs[i] = Vector.addVectors(realVecs[i], rotVecs[i].negate());
        }

        Vector[] final1Vecs = checkVCriteria(resultVecs);
        Vector[] final2Vecs = new Vector[final1Vecs.length];
        for(int i = 0; i < final1Vecs.length; i++){
            if(final1Vecs[i] != null){
                final2Vecs[i] = final1Vecs[i];
            } else {
                badWheels++;
            }
        }
        botLocation.add(Vector.averageVectors(final2Vecs));
    }

    public Vector[] checkVCriteria(Vector[] vecs){
        Vector[] output = vecs;

        //calculate mean
        double averageX = 0;
        double averageY = 0;
        for(Vector v: output){
            averageX += v.getX();
            averageY += v.getY();
        }
        averageX = averageX / output.length;
        averageY = averageY / output.length;

        //calculate standard deviation
        double sumX = 0;
        double sumY = 0;
        for(Vector v: output){
            sumX += (v.getX() - averageX) * (v.getX() - averageX);
            sumY += (v.getY() - averageY) * (v.getY() - averageY);
        }
        double stanDeviationX = Math.sqrt(sumX / output.length);
        double stanDeviationY = Math.sqrt(sumY / output.length);

        SmartDashboard.putNumber("stDev X: ", stanDeviationX);
        SmartDashboard.putNumber("stDev Y: ", stanDeviationY);

        //weed out bad wheels by checking how many standard deviations they are away from the mean
        double devX = stanDeviationX * cals.maxStandardDeviations;
        double devY = stanDeviationY * cals.maxStandardDeviations;
        for(int i = 0; i < output.length; i++){
            if(Math.abs(averageX - output[i].getX()) > devX || averageY - output[i].getY() > devY){
                output[i] = null;
            }
        }

        return output;
    }

    @Override
    public void close(){

    }
}
