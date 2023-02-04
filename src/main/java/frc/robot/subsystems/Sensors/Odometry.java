package frc.robot.subsystems.Sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.subsystems.Drive.DriveCal.WheelCal;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class Odometry implements AutoCloseable {
    
    OdometryCals cals;
    public WheelCal[] wCal;

    public Vector botLocation;
    public double botAngle;

    public Odometry(OdometryCals cals, WheelCal[] wCal){
        this.cals = cals;
        this.wCal = wCal;

        setBotLocation(Vector.fromXY(0, 0));
        botAngle = 0;
    }

    public void setBotLocation(Vector location){
        botLocation = location;
    }

    public void setBotAngle(double angle){
        botAngle = angle;
    }

    public void setBotPose(AutonPos pos){
        setBotLocation(pos.xy);
        setBotAngle(pos.theta);
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

        getWheelDiffsOdo(botAng, prevBotAng, realVecs);
        //getStdDevOdo(botAng, realVecs);
        
        prevWheelStates = wheelStates;
        prevBotAng = botAng;
    }

    /* This iteration of odometry looks at the real wheel values and formulates
     * a rigidbody between the various wheels. It measures error by the difference 
     * between what each wheel thinks the center point is.
     */
    public double totalError = 0;
    double prevAng = 999;
    public void getWheelDiffsOdo(double navXBotAng, double prevNavXBotAng, Vector[] realVecs){
        if(prevAng == 999){
            prevAng = botAngle;
        }

        Vector[] wheelLocations = new Vector[realVecs.length];
        for(int i = 0; i < wheelLocations.length; i++){
            wheelLocations[i] = wCal[i].wheelLocation;
        }

        double[] bestValues = formulateBestValues(realVecs, wheelLocations);
        Vector bestStrafe = new Vector(bestValues[0], bestValues[1]);
        double bestAngle = bestValues[2];
        double error = bestValues[3];
        
        //Send out a total error for a +- value
        totalError += error;
        SmartDashboard.putNumber("Rotation Error", totalError);

        //Set the global pos & ang values
        bestStrafe.theta += botAngle;//Convert to field relative

        if(Math.abs(navXBotAng - prevNavXBotAng) + cals.maxNavXWheelAngDiff < Math.abs(bestAngle)){
            botAngle += bestAngle;
        } else {
            botAngle += (navXBotAng - prevNavXBotAng);
        }
        botLocation.add(bestStrafe);

        prevAng = botAngle;
    }

    static int[][] groups = {{0,1},
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
    
    //Returns r, theta, angle, error
    public static double[] formulateBestValues(Vector[] realVecs, Vector[] wheelLocations){
        double error = 0;

        double minError = Double.POSITIVE_INFINITY;//Yuh
        Vector bestStrafe = new Vector(0, 0);
        double bestAngle = 0;

        for(int[] group : groups){

            //Gets new wheel positions by adding the vector from the center and how far the wheel actually moved
            Vector[] wheelPos = new Vector[group.length];
            for(int i = 0; i < group.length; i++){
                wheelPos[i] = Vector.addVectors(wheelLocations[group[i]], realVecs[group[i]]);
            }
            
            //Difference between that index and the wheel directly before it
            Vector[] wheelDiffs = new Vector[group.length];
            Vector[] realWheelDiffs = new Vector[group.length];
            int prevIdx = group.length - 1;
            for(int i = 0; i < group.length; i++){
                wheelDiffs[i] = Vector.subVector(wheelPos[i], wheelPos[prevIdx]);
                realWheelDiffs[i] = Vector.subVector(wheelLocations[group[i]], wheelLocations[group[prevIdx]]);

                prevIdx = i;
            }

            //Formulates new wheel locations based on an adjusted robot center
            //Then makes individually calculated strafes and angles for each wheel's movement
            Vector[] newWheelLocations = new Vector[group.length];

            Vector[] strafes = new Vector[group.length];
            double[] angles = new double[group.length];
            for(int i = 0; i < group.length; i++){
                newWheelLocations[i] = new Vector(wheelLocations[group[i]].r, wheelLocations[group[i]].theta + (wheelDiffs[i].theta - realWheelDiffs[i].theta));

                strafes[i] = Vector.subVector(wheelPos[i], newWheelLocations[i]);

                angles[i] = wheelDiffs[i].theta - realWheelDiffs[i].theta;
            }

            //Put these all into one double by adding all of them
            double finalAng = 0;
            for(int i = 0; i < group.length; i++){
                finalAng += angles[i];
            }

            //calculate error by formulating every possible vector between the wheels in the group
            int prevErrorIdx = group.length - 1;
            double wheelError = 0;
            int i = 0;
            for( ; i < group.length - 1; i++){
                wheelError += Vector.averageVectors(strafes[i], strafes[prevErrorIdx]).r;

                prevErrorIdx = i;
            }
            wheelError /= i;

            //String s = "";
            //for(int iii : group){
            //    s += iii + " ";
            //}
            //System.out.println("Group: " + s);
            //System.out.println("Strafe: " + Vector.averageVectors(strafes));
            //System.out.println("Angle: " + finalAng / group.length);
            //System.out.println("Wheel Error: " + wheelError);
            //System.out.println();

            //Set the best values based on error calculation
            if(wheelError < minError){
                bestStrafe = Vector.averageVectors(strafes);
                bestAngle = Angle.normRad(finalAng) / group.length;
                error = wheelError / group.length;

                minError = wheelError;
            }
        }

        return new double[] {bestStrafe.r, bestStrafe.theta, bestAngle, error};
    }

    /* This iteration of odometry looks at the wheels as a whole and how much they
     * individually are different from the rest. We calculate the standard
     * deviation of the group of 4, and then each wheel that is a calibratable number
     * of standard deviations away from the average is ignored entirely.
     */
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

        Vector[] final1Vecs = checkVStDevCriteria(resultVecs);
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

    public Vector[] checkVStDevCriteria(Vector[] vecs){
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
