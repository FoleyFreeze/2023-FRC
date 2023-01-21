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

    public Odometry(OdometryCals cals, WheelCal[] wCal){
        this.cals = cals;
        this.wCal = wCal;

        setBotLocation(Vector.fromXY(0, 0));
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

        double deltaAng = Angle.toRad(botAng - prevBotAng);
        
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
            double rotAng = wCal[i].wheelLocation.theta + Math.PI / 2.0;
            double r = wCal[i].wheelLocation.r * deltaAng;
            rotVecs[i] = new Vector(r, rotAng);
        }

        //subtract the rotation vectors away from the originals
        Vector[] resultVecs = new Vector[wheelNum];
        for(int i = 0; i < wheelNum; i++){
            resultVecs[i] = Vector.addVectors(strafeVecs[i], rotVecs[i].negate());
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

        prevWheelStates = wheelStates;
        prevBotAng = botAng;
    }

    public void setBotLocation(Vector location){
        botLocation = location;
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
