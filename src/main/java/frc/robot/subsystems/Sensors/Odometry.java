package frc.robot.subsystems.Sensors;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.subsystems.Drive.DriveCal.WheelCal;
import frc.robot.util.Angle;
import frc.robot.util.LimitedStack;
import frc.robot.util.Vector;

public class Odometry implements AutoCloseable {
    
    OdometryCals cals;
    public WheelCal[] wCal;

    public Vector botLocation;
    public double botAngle;

    RobotContainer r;

    public class OldLocation{
        double time;
        Vector space;
        double angle;
    }

    public OldLocation getOldRobotLocation(double time){
        OldLocation prevLoc = oldLocations.peek();
        OldLocation currLoc = prevLoc;
        for(OldLocation loc: oldLocations){
            currLoc = loc;
            if(loc.time < time){
                break;
            }
            prevLoc = currLoc;
        }

        double x = (time - prevLoc.time) / (currLoc.time - prevLoc.time);
        
        OldLocation newLocation = new OldLocation();
        newLocation.time = time;
        double angleDiff = currLoc.angle - prevLoc.angle;
        newLocation.angle = x * angleDiff + prevLoc.angle;
        Vector vecDiff = Vector.subVectors(currLoc.space, prevLoc.space);
        vecDiff.r *= x;
        vecDiff.add(prevLoc.space);
        newLocation.space = vecDiff;
        
        return newLocation;
    }
    LimitedStack<OldLocation> oldLocations;

    public Odometry(OdometryCals cals, WheelCal[] wCal, RobotContainer r){
        this.cals = cals;
        this.wCal = wCal;
        this.r = r;

        oldLocations = new LimitedStack<>(26);
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
        setBotAngle(pos.value);
    }

    public int badWheels = 0;
    private Vector[] prevWheelStates;
    public double prevBotAng;
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
                wheelAngle = ((wheelStates[i].theta) + (prevWheelStates[i].theta)) / 2.0;
            } else {
                wheelAngle = wheelStates[i].theta;
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
    public double totalXError = 0;
    public double totalYError = 0;
    public double totalAngError = 0;
    public double totalStDevCor = 0;
    double odoAngle = 0;
    double timeOfError = 0;
    public void getWheelDiffsOdo(double navXBotAng, double prevNavXBotAng, Vector[] realVecs){

        Vector[] wheelLocations = {wCal[0].wheelLocation,wCal[1].wheelLocation,wCal[2].wheelLocation,wCal[3].wheelLocation};

        /*
        for(Vector v : realVecs){
            double tr = v.r * 1000;
            tr = Math.round(tr);
            double tt = v.theta * 1000;
            tt = Math.round(tt);
            v.r = tr / 1000;
            v.theta = tt / 1000;
        }
        */

        double[] bestValues;
        try{
            bestValues = formulateBestValuesMatrix(realVecs, wheelLocations, r.sensors.getPitch(), r.sensors.getRoll());
        } catch(Exception e){
            if(Timer.getFPGATimestamp() - timeOfError > 5){
                timeOfError = Timer.getFPGATimestamp();
                e.printStackTrace();
            }
            bestValues = new double[] {0,0,0,0,0,0,0};
        }
        Vector bestStrafe = new Vector(bestValues[0], bestValues[1]);
        double bestAngle = bestValues[2];
        double xError = bestValues[3];
        double yError = bestValues[4];
        double angError = bestValues[5];

        if(DriverStation.isEnabled()){
            /*     
            System.out.format("%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",Timer.getFPGATimestamp(),
            realVecs[0].r,realVecs[0].theta,
            realVecs[1].r,realVecs[1].theta,
            realVecs[2].r,realVecs[2].theta,
            realVecs[3].r,realVecs[3].theta,
            bestStrafe.getX(),bestStrafe.getY());
            */
            
            //System.out.format("%f,%.18f,%.18f,%.18f,%.18f,%.18f,%.18f,%.18f,%.18f,%f,%f,%f\n",Timer.getFPGATimestamp(),realVecs[0].getX(),realVecs[0].getY(),realVecs[1].getX(),realVecs[1].getY(),realVecs[2].getX(),realVecs[2].getY(),realVecs[3].getX(),realVecs[3].getY(),Angle.toDeg(navXBotAng),bestStrafe.getX(),bestStrafe.getY());
        }
        
        //Send out a total error for a +- value
        totalXError += xError;
        totalYError += yError;
        totalAngError += angError;
        totalStDevCor += bestValues[6];
        //SmartDashboard.putNumber("+/- x-position", totalXError);
        //SmartDashboard.putNumber("+/- y-position", totalYError);
        //SmartDashboard.putNumber("+/- angle", totalAngError);

        //Set the global pos & ang values
        bestStrafe.theta += botAngle;//Convert to field relative

        /*if(Math.abs(navXBotAng - prevNavXBotAng) + cals.maxNavXWheelAngDiff < Math.abs(bestAngle)){
            botAngle += bestAngle;
        } else {
            botAngle += (navXBotAng - prevNavXBotAng);
        }*/
        botAngle = navXBotAng;//+= bestAngle;
        botLocation.add(bestStrafe);
        odoAngle += bestAngle;
        odoAngle %= 2*Math.PI;
        SmartDashboard.putNumber("OdoAngle", Math.toDegrees(odoAngle));

        OldLocation o = new OldLocation();
        o.time = Timer.getFPGATimestamp();
        o.space = new Vector(botLocation);
        o.angle = botAngle;
        oldLocations.push(o);
    }

    //Returns r, theta, angle, x error, y error, angle error
    public static double[] formulateBestValuesMatrix(Vector[] realVecs, Vector[] wheelLocations, double pitch, double roll){
        Vector[] normalVecs = new Vector[realVecs.length];
        double normalVecMean = 0;
        boolean thetaCheck = true;
        for(int i=0;i<realVecs.length;i++){
            Vector v = realVecs[i];
            normalVecs[i] = new Vector(1,v.theta+Math.PI/2);
            if(i>0){
                thetaCheck &= normalVecs[i].theta == normalVecs[i-1].theta;
            }

            //find mean angle
            if(i == 0){
                normalVecMean = normalVecs[i].theta % (2*Math.PI);
            } else {
                double tdiff = normalVecs[i].theta - normalVecMean;
                if(tdiff > Math.PI){
                    tdiff -= 2*Math.PI;
                } else if(tdiff < -Math.PI){
                    tdiff += 2*Math.PI;
                }
                normalVecMean += tdiff/(i+1);
                normalVecMean %= 2*Math.PI;
            }
        }
        
        if(thetaCheck){
            normalVecs[0].theta += 1e-5;
        }
        double sumOfDiff = 0;
        for(Vector v: normalVecs){
            double tdiff = v.theta - normalVecMean;
            if(tdiff > Math.PI){
                tdiff -= 2*Math.PI;
            } else if(tdiff < -Math.PI){
                tdiff += 2*Math.PI;
            }
            sumOfDiff += Math.abs(tdiff);
        }
        Vector centerOfRot = getLeastSquaresCenterOfRotation(normalVecs, wheelLocations);
        //if we get a bad center and all angles are similar, try again
        double centerThrsh = 250;
        if(sumOfDiff < 0.01 && centerOfRot.r < centerThrsh){
            normalVecs[0].theta += 1e-3;
            centerOfRot = getLeastSquaresCenterOfRotation(normalVecs, wheelLocations);
            if(centerOfRot.r < centerThrsh){
                //if center is still bad, then use a manual one
                centerOfRot = new Vector(1e6,normalVecMean);
            }
        }
        
        SmartDashboard.putString("Center Of Rot", centerOfRot.toString());

        //angle logic
        double[] angles = new double[realVecs.length];
        for(int i = 0; i < realVecs.length; i++){
            Vector radius = Vector.subVectors(centerOfRot, wheelLocations[i]);
            if(realVecs[i].cross2D(radius) > 0){
                angles[i] = (realVecs[i].r/radius.r);
            } else {
                angles[i] = -(realVecs[i].r/radius.r);
            }
        }

        //strafe logic
        Vector[] strafes = new Vector[realVecs.length];
        for(int i = 0; i < realVecs.length; i++){
            Vector newBotCenter = new Vector(centerOfRot.r, centerOfRot.theta - angles[i]);
            strafes[i] = Vector.subVectors(newBotCenter, centerOfRot);
        }

        //Bad wheel detection
        double strafeStDev = getStDev(strafes);

        //hacky bs
        Vector[] vectorAngles = new Vector[angles.length];
        for(int i = 0; i < angles.length; i++){
            vectorAngles[i] = new Vector(angles[i], 0);
        }
        //Ironically, you only really want to look at the magnitude instead of angle to find the angles
        //get rid of repeat values
        double angleStDev = getStDev(vectorAngles);

        boolean[] badValues = {false, false, false, false};//true means it's bad

        //double bestStrafeDiff = Double.POSITIVE_INFINITY;
        int[] bestStrafePos = new int[2];
        //double bestAngleDiff = Double.POSITIVE_INFINITY;
        //int[] bestAngPos = new int[2];

        //Find the best values we have
        if(Math.abs(pitch) > Math.abs(roll)){
            if(pitch > 0){
                bestStrafePos = new int[]{2, 3};
            } else {
                bestStrafePos = new int[]{0, 1};
            }
        } else {
            if(roll > 0){
                bestStrafePos = new int[]{1, 2};
            } else {
                bestStrafePos = new int[]{0, 3};
            }
        }
        /*for(int i = 0; i < strafes.length; i++){
            for(int compareIdx = 1; compareIdx < strafes.length-1; compareIdx++){
                int idxWrapper = (i + compareIdx) % strafes.length;
                if(Math.abs(Vector.subVectors(strafes[i], strafes[idxWrapper]).r) < bestStrafeDiff){
                    bestStrafeDiff = Math.abs(Vector.subVectors(strafes[i], strafes[idxWrapper]).r);
                    bestStrafePos[0] = i;
                    bestStrafePos[1] = idxWrapper;
                }
                if(Math.abs(strafes[i].r - strafes[idxWrapper].r) < bestAngleDiff){
                    bestAngleDiff = Math.abs(strafes[i].r - strafes[idxWrapper].r);
                    bestAngPos[0] = i;
                    bestAngPos[1] = idxWrapper;
                }
            }
        }*/

        Vector bestStrafeVal = Vector.averageVectors(strafes[bestStrafePos[0]], strafes[bestStrafePos[1]]);
        //double bestAngVal = (strafes[bestStrafePos[0]].r + strafes[bestStrafePos[1]].r) / 2;
        //remove the bad wheels and their no-good values
        
        for(int i = 0; i < strafes.length; i++){
            if(Math.abs(strafeStDev) > 0.01 && i != bestStrafePos[0] && i != bestStrafePos[1] &&
               Math.abs(Vector.subVectors(strafes[i], bestStrafeVal).r) > strafeStDev * OdometryCals.maxStandardDeviationsStrafeCOR){
                badValues[i] = true;
            }
            /*if(Math.abs(angleStDev) > 0.01 && i != bestAngPos[0] && i != bestAngPos[1] &&
               Math.abs(strafes[i].r - bestAngVal) > angleStDev * OdometryCals.maxStandardDeviationsAngleCOR){
                badValues[i] = true;
            }*/
        }

        double highestStrafeX = Double.NEGATIVE_INFINITY;
        double highestStrafeY = Double.NEGATIVE_INFINITY;
        double lowestStrafeX = Double.POSITIVE_INFINITY;
        double lowestStrafeY = Double.POSITIVE_INFINITY;

        double highestAngle = Double.NEGATIVE_INFINITY;
        double lowestAngle = Double.POSITIVE_INFINITY;

        double finalStrafeX = 0;
        double finalStrafeY = 0;
        double finalAngle = 0;
        double finalLength = 0;
        for(int i = 0; i < realVecs.length; i++){
            if(badValues[i] == false){
                double x = strafes[i].getX();
                double y = strafes[i].getY();
                double angle = vectorAngles[i].r;
                if(vectorAngles[i].theta > Math.PI/2) angle = -angle;

                finalStrafeX += x;
                if(x >= highestStrafeX) highestStrafeX = x;
                if(x <= lowestStrafeX) lowestStrafeX = x;

                finalStrafeY += y;
                if(y >= highestStrafeY) highestStrafeY = y;
                if(y <= lowestStrafeY) lowestStrafeY = y;

                finalAngle += angle;
                if(angle >= highestAngle) highestAngle = angle;
                if(angle <= lowestAngle) lowestAngle = angle;
                
                finalLength++;
            }
        }
        if(finalLength == 0) finalLength = 1;
        Vector finalStrafe = Vector.fromXY(finalStrafeX/finalLength, finalStrafeY/finalLength);
        finalAngle /= finalLength;

        //The +/- values on where we think we could be
        double xError = (highestStrafeX - lowestStrafeX) / 2;
        double yError = (highestStrafeY - lowestStrafeY) / 2;
        double angError = (highestAngle - lowestAngle) / 2;

        return new double[] {finalStrafe.r, finalStrafe.theta, finalAngle, xError, yError, angError, centerOfRot.getX(), centerOfRot.getY()};
    }

    //this builds off the idea that even if slipping, we still expect the wheel angles to be in a valid swerve config
    //because of that we can create a singular best estimate of the COR and then select from all the resulting
    //individual wheel strafes and angles to determine if any wheels are slipping
    static Vector getLeastSquaresCenterOfRotation(Vector[] normalVecs, Vector[] locationVecs){

        SimpleMatrix A = new SimpleMatrix(4,2);
        SimpleMatrix b = new SimpleMatrix(4,1);
        for(int i=0;i<normalVecs.length;i++){
            double dx = normalVecs[i].getY();
            double dy = normalVecs[i].getX();
            double x = locationVecs[i].getX();
            double y = locationVecs[i].getY();

            A.set(i,0,dx);
            A.set(i,1,-dy);

            b.set(i, x*dx - y*dy);
        }

        //solve matrix equation via p = inv(A)*b
        //borrow the matrix library WPILib already uses for this
        SimpleMatrix out = A.pseudoInverse().mult(b);

        Vector point = Vector.fromXY(out.get(0),out.get(1));

        return point;
    }

    static Vector[] averageVectorArray(Vector[][] vecs){
        Vector[] result = new Vector[vecs.length];
        for(int i = 0; i < vecs.length; i++){
            result[i] = Vector.averageVectors(vecs[i]);
        }

        return result;
    }

    static double getStDev(Vector[] vecs){
        //calculate mean
        double averageX = 0;
        double averageY = 0;
        for(Vector v: vecs){
            averageX += v.getX();
            averageY += v.getY();
        }
        averageX = averageX / vecs.length;
        averageY = averageY / vecs.length;

        //calculate standard deviation
        double sumX = 0;
        double sumY = 0;
        for(Vector v: vecs){
            sumX += (v.getX() - averageX) * (v.getX() - averageX);
            sumY += (v.getY() - averageY) * (v.getY() - averageY);
        }
        double stanDeviationX = Math.sqrt(sumX / vecs.length);
        double stanDeviationY = Math.sqrt(sumY / vecs.length);

        return Vector.fromXY(stanDeviationX, stanDeviationY).r;
    }

    //Must have 12 total values or less
    static Vector[] condenseArrayArray(Vector[][] vecs){
        Vector[] result = new Vector[vecs.length * vecs[0].length];
        for(int i = 0; i < result.length; i++){
            int wheelIdx;
            if(i <= 2){
                wheelIdx = 0;
            } else if(i <= 5){
                wheelIdx = 1;
            } else if(i <= 8){
                wheelIdx = 2;
            } else {
                wheelIdx = 3;
            }
            int centerIdx = (i % vecs[0].length);
            result[i] = vecs[wheelIdx][centerIdx];
        }
        return result;
    }

    //Returns r, theta, angle, x error, y error, angle error
    public static double[] formulateBestValues(Vector[] realVecs, Vector[] wheelLocations){
        Vector[][] centersOfRot = formulateCentersOfRot(realVecs, wheelLocations);
        if(DriverStation.isEnabled() || Robot.isSimulation()){
            String s = "";
            for(Vector[] vv : centersOfRot){
                for(Vector v : vv){
                    s += String.format("%.2f,%.2f | ",v.getX(),v.getY());
                }
                s += "\n";
            }
            System.out.println(s);
        }


        Vector[] allCenters = condenseArrayArray(centersOfRot);
        double stDevCORs = getStDev(allCenters);

        //angle logic
        double[][] angles = new double[centersOfRot.length][centersOfRot[0].length];
        for(int i = 0; i < realVecs.length; i++){
            for(int corIdx = 0; corIdx < 3; corIdx++){
                Vector radius = Vector.subVectors(wheelLocations[i], centersOfRot[i][corIdx]);
                if(true || radius.r < 10000000.0){
                    angles[i][corIdx] = (realVecs[i].r/radius.r);
                } else {
                    angles[i][corIdx] = 0;
                }
            }
        }

        //strafe logic
        Vector[][] strafes = new Vector[centersOfRot.length][centersOfRot[0].length];
        for(int i = 0; i < realVecs.length; i++){
            for(int corIdx = 0; corIdx < 3; corIdx++){
                Vector cor = centersOfRot[i][corIdx];
                Vector newBotCenter = new Vector(cor.r, cor.theta + angles[i][corIdx]);
                if(true || cor.r < 10000000.0){
                    strafes[i][corIdx] = Vector.subVectors(newBotCenter, cor);
                } else {
                    strafes[i][corIdx] = new Vector(realVecs[i]);
                }
            }
        }

        //Bad wheel detection
        Vector[] allCondensedStrafes = condenseArrayArray(strafes);
        //get rid of repeat values
        Vector[] condensedStrafes = {Vector.averageVectors(allCondensedStrafes[0], allCondensedStrafes[1], allCondensedStrafes[2]),
                                     Vector.averageVectors(allCondensedStrafes[3], allCondensedStrafes[4], allCondensedStrafes[5]),
                                     Vector.averageVectors(allCondensedStrafes[5], allCondensedStrafes[7], allCondensedStrafes[8]),
                                     Vector.averageVectors(allCondensedStrafes[9], allCondensedStrafes[10], allCondensedStrafes[11])};
        double strafeStDev = getStDev(condensedStrafes);

        //hacky bs
        Vector[][] vectorAngles = new Vector[angles.length][angles[0].length];
        for(int i = 0; i < angles.length; i++){
            for(int corIdx = 0; corIdx < angles[0].length; corIdx++){
                vectorAngles[i][corIdx] = new Vector(angles[i][corIdx], 0);
            }
        }
        Vector[] allCondensedAngles = condenseArrayArray(vectorAngles);//Ironically, you only really want to look at the magnitude instead of angle to find the angles
        //get rid of repeat values
        Vector[] condensedAngles = {Vector.averageVectors(allCondensedAngles[0], allCondensedAngles[1], allCondensedAngles[2]),
                                    Vector.averageVectors(allCondensedAngles[3], allCondensedAngles[4], allCondensedAngles[5]),
                                    Vector.averageVectors(allCondensedAngles[5], allCondensedAngles[7], allCondensedAngles[8]),
                                    Vector.averageVectors(allCondensedAngles[9], allCondensedAngles[10], allCondensedAngles[11])};
        double angleStDev = getStDev(condensedAngles);


        boolean[] badValues = {false, false, false, false};//true means it's bad

        double bestStrafeDiff = Double.POSITIVE_INFINITY;
        int[] bestStrafePos = new int[2];
        double bestAngleDiff = Double.POSITIVE_INFINITY;
        int[] bestAngPos = new int[2];

        //Find the best values we have
        for(int i = 0; i < condensedStrafes.length; i++){
            for(int compareIdx = 1; compareIdx < condensedStrafes.length-1; compareIdx++){
                int idxWrapper = (i + compareIdx) % condensedStrafes.length;
                if(Math.abs(Vector.subVectors(condensedStrafes[i], condensedStrafes[idxWrapper]).r) < bestStrafeDiff){
                    bestStrafeDiff = Math.abs(Vector.subVectors(condensedStrafes[i], condensedStrafes[idxWrapper]).r);
                    bestStrafePos[0] = i;
                    bestStrafePos[1] = idxWrapper;
                }
                if(Math.abs(condensedAngles[i].r - condensedAngles[idxWrapper].r) < bestAngleDiff){
                    bestAngleDiff = Math.abs(condensedAngles[i].r - condensedAngles[idxWrapper].r);
                    bestAngPos[0] = i;
                    bestAngPos[1] = idxWrapper;
                }
            }
        }

        Vector bestStrafeVal = Vector.averageVectors(condensedStrafes[bestStrafePos[0]], condensedStrafes[bestStrafePos[1]]);
        double bestAngVal = (condensedAngles[bestStrafePos[0]].r + condensedAngles[bestStrafePos[1]].r) / 2;
        //remove the bad wheels and their no-good values
        
        for(int i = 0; i < condensedStrafes.length; i++){
            if(Math.abs(strafeStDev) > 0.01 && i != bestStrafePos[0] && i != bestStrafePos[1] &&
               Math.abs(Vector.subVectors(condensedStrafes[i], bestStrafeVal).r) > strafeStDev * OdometryCals.maxStandardDeviationsStrafeCOR){
                badValues[i] = true;
            }
            if(Math.abs(angleStDev) > 0.01 && i != bestAngPos[0] && i != bestAngPos[1] &&
               Math.abs(condensedAngles[i].r - bestAngVal) > angleStDev * OdometryCals.maxStandardDeviationsAngleCOR){
                badValues[i] = true;
            }
        }

        double highestStrafeX = Double.NEGATIVE_INFINITY;
        double highestStrafeY = Double.NEGATIVE_INFINITY;
        double lowestStrafeX = Double.POSITIVE_INFINITY;
        double lowestStrafeY = Double.POSITIVE_INFINITY;

        double highestAngle = Double.NEGATIVE_INFINITY;
        double lowestAngle = Double.POSITIVE_INFINITY;

        double finalStrafeX = 0;
        double finalStrafeY = 0;
        double finalAngle = 0;
        double finalLength = 0;
        for(int i = 0; i < realVecs.length; i++){
            if(badValues[i] == false){
                double x = condensedStrafes[i].getX();
                double y = condensedStrafes[i].getY();
                double angle = condensedAngles[i].r;

                finalStrafeX += x;
                if(x >= highestStrafeX) highestStrafeX = x;
                if(x <= lowestStrafeX) lowestStrafeX = x;

                finalStrafeY += y;
                if(y >= highestStrafeY) highestStrafeY = y;
                if(y <= lowestStrafeY) lowestStrafeY = y;

                finalAngle += angle;
                if(angle >= highestAngle) highestAngle = angle;
                if(angle <= lowestAngle) lowestAngle = angle;
                
                finalLength++;
            }
        }
        if(finalLength == 0) finalLength = 1;
        Vector finalStrafe = Vector.fromXY(finalStrafeX/finalLength, finalStrafeY/finalLength);
        finalAngle /= finalLength;

        //The +/- values on where we think we could be
        double xError = (highestStrafeX - lowestStrafeX) / 2;
        double yError = (highestStrafeY - lowestStrafeY) / 2;
        double angError = (highestAngle - lowestAngle) / 2;

        return new double[] {finalStrafe.r, finalStrafe.theta, finalAngle, xError, yError, angError, stDevCORs};
    }

    public static Vector getIntersection(Vector v1, Vector v2, Vector v3, Vector v4){
        double y1 = v1.getY();
        double y2 = v2.getY();
        double y3 = v3.getY();
        double y4 = v4.getY();

        double x1 = v1.getX();
        double x2 = v2.getX();
        double x3 = v3.getX();
        double x4 = v4.getX();

        double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        
        double corX = (((x1 * y2) - (x2 * y1)) * (x3 - x4)) - (((x3 * y4) - (x4 * y3)) * (x1 - x2));
        double corY = (((x1 * y2) - (x2 * y1)) * (y3 - y4)) - (((x3 * y4) - (x4 * y3)) * (y1 - y2));

        System.out.format("%.3f,%.0f,%.0f\n",d,corX/d,corY/d);

        return Vector.fromXY(corX/d, corY/d);
    }

    public static Vector[][] formulateCentersOfRot(Vector[] realVecs, Vector[] wheelLocations){

        Vector[] newWheelLocations = new Vector[wheelLocations.length];
        Vector[] perps = new Vector[newWheelLocations.length];
        for(int i = 0; i < newWheelLocations.length; i++){
            newWheelLocations[i] = Vector.addVectors(wheelLocations[i], realVecs[i]);

            Vector perpMovement = new Vector(realVecs[i]);
            perpMovement.theta += Math.PI/2;
            perps[i] = Vector.addVectors(wheelLocations[i], perpMovement);
        }

        Vector[][] centersOfRot = new Vector[wheelLocations.length][3];
        for(int i = 0; i < wheelLocations.length; i++){
            for(int corIdx = 1; corIdx <= 3; corIdx++){
                int idxWrapper = (i + corIdx) % (wheelLocations.length);

                //Via this equation
                //https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
                double y1 = wheelLocations[i].getY();
                double y2 = perps[i].getY();
                double y3 = wheelLocations[idxWrapper].getY();
                double y4 = perps[idxWrapper].getY();

                double x1 = wheelLocations[i].getX();
                double x2 = perps[i].getX();
                double x3 = wheelLocations[idxWrapper].getX();
                double x4 = perps[idxWrapper].getX();

                double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
                
                double corX = (((x1 * y2) - (x2 * y1)) * (x3 - x4)) - (((x3 * y4) - (x4 * y3)) * (x1 - x2));
                double corY = (((x1 * y2) - (x2 * y1)) * (y3 - y4)) - (((x3 * y4) - (x4 * y3)) * (y1 - y2));
                
                if(Math.abs(d) < 0.0){
                    double movementAngle = realVecs[i].theta;
                    double otherWheelMovementAngle = realVecs[idxWrapper].theta;

                    if(Math.abs(movementAngle - Angle.normRad(otherWheelMovementAngle - Math.PI)) < 0.1){//Parallel angles in a rotation
                        centersOfRot[i][corIdx-1] = Vector.fromXY(0, 0);
                    } else {//parallel angles in a strafe
                        centersOfRot[i][corIdx-1] = new Vector(1e9, movementAngle + Math.PI/2);
                    }
                } else {
                    corX /= d;
                    corY /= d;
                    centersOfRot[i][corIdx-1] = Vector.fromXY(corX, corY);
                }
            }
        }
        
        return centersOfRot;
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
    public static double[] formulateOkValues(Vector[] realVecs, Vector[] wheelLocations){
        double error = 0;

        double minError = Double.POSITIVE_INFINITY;//Yuh
        Vector bestStrafe = new Vector(0, 0);
        double bestAngle = 0;

        double bestGroup = 0;
        double groupidx = -1;
        for(int[] group : groups){
            groupidx++;

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
                wheelDiffs[i] = Vector.subVectors(wheelPos[i], wheelPos[prevIdx]);
                realWheelDiffs[i] = Vector.subVectors(wheelLocations[group[i]], wheelLocations[group[prevIdx]]);

                prevIdx = i;
            }

            //Formulates new wheel locations based on an adjusted robot center
            //Then makes individually calculated strafes and angles for each wheel's movement
            Vector[] newWheelLocations = new Vector[group.length];

            Vector[] strafes = new Vector[group.length];
            double[] angles = new double[group.length];
            for(int i = 0; i < group.length; i++){
                angles[i] = wheelDiffs[i].theta - realWheelDiffs[i].theta;

                newWheelLocations[i] = new Vector(wheelLocations[group[i]].r, wheelLocations[group[i]].theta + angles[i]);

                strafes[i] = Vector.subVectors(wheelPos[i], newWheelLocations[i]);
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
                wheelError += Vector.subVectors(strafes[i], strafes[prevErrorIdx]).r;

                prevErrorIdx = i;
            }
            wheelError /= i;

            String s = "";
            for(int iii : group){
                s += iii + " ";
            }
            System.out.println("Group: " + s);
            System.out.println("Strafe: " + Vector.averageVectors(strafes));
            System.out.println("Angle: " + finalAng / group.length);
            System.out.println("Wheel Error: " + wheelError);
            System.out.println();

            //Set the best values based on error calculation
            if(wheelError < minError){
                bestStrafe = Vector.averageVectors(strafes);
                bestAngle = Angle.normRad(finalAng / group.length);
                error = wheelError / group.length;
                bestGroup = groupidx;
                minError = wheelError;
            }
        }

        return new double[] {bestStrafe.r, bestStrafe.theta, bestAngle, error, bestGroup};
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
        Vector deltaStrafe = Vector.averageVectors(final2Vecs);
        deltaStrafe.theta -= botAngle;
        botLocation.add(Vector.averageVectors(deltaStrafe));

        botAngle += Angle.normRad(botAng - prevBotAng);
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

        //weed out bad wheels by checking how many standard deviations they are away from the mean
        double devX = stanDeviationX * cals.maxStandardDeviations;
        double devY = stanDeviationY * cals.maxStandardDeviations;
        for(int i = 0; i < output.length; i++){
            if(Math.abs(averageX - output[i].getX()) > devX || Math.abs(averageY - output[i].getY()) > devY){
                output[i] = null;
            }
        }

        return output;
    }

    @Override
    public void close(){

    }
}
