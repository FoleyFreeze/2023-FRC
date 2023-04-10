package frc.robot.commands.Auton.AdvancedMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.commands.Auton.AutonCal.MPCals;
import frc.robot.util.Vector;

public class MultiDimensionalMotionProfile extends CommandBase {

    public RobotContainer r;
    MPCals mpCals;

    Vector startLoc;
    double startAngle;

    AutonPos[] wayPoints;

    final double MIN_R;

    public MultiDimensionalMotionProfile(RobotContainer r, MPCals mpCals, AutonPos... wayPoints){
        this.r = r;
        this.mpCals = mpCals;

        this.wayPoints = wayPoints;

        MIN_R = r.driveTrain.cals.wheelCals[0].wheelLocation.r;
        addRequirements(r.driveTrain);
    }

    /*makes new before/after arc points that are a wheel location
     * distance away from the hard midpoint; the values of each
     * autonpos object are equal to the lengths of their respective 
     * following segments
     */
    public static AutonPos[] formulateArcs(double minR, Vector startPos, AutonPos... wayPoints){
        //add all waypoints + start position to a new array
        Vector[] pointsWStart = new Vector[wayPoints.length + 1];
        pointsWStart[0] = startPos;
        for(int i = 1; i < pointsWStart.length; i++){
            pointsWStart[i] = wayPoints[i-1].xy;
        }

        Vector[] points = new Vector[(wayPoints.length - 1) * 2 + 2];
        points[0] = new Vector(startPos);

        for(int i = 0; i < pointsWStart.length - 2; i++){
            Vector firstPoint = pointsWStart[i];
            Vector midPoint = pointsWStart[i+1];
            Vector lastPoint = pointsWStart[i+2];

            Vector firstVec = Vector.subVectors(midPoint, firstPoint);
            Vector lastVec = Vector.subVectors(lastPoint, midPoint);

            //adds the complementary angles
            double angle = (Math.PI / 2 - firstVec.theta) + (Math.PI / 2 - lastVec.theta);

            double dist;
            if(angle != Math.PI){
                dist = minR / Math.tan(angle / 2);
            } else {
                dist = 0;
            }

            points[i+1] = Vector.subVectors(midPoint, new Vector(dist, firstVec.theta));
            points[i+2] = Vector.subVectors(lastPoint, new Vector(lastVec.r - dist, lastVec.theta));
        }
        
        points[points.length-1] = new Vector(pointsWStart[pointsWStart.length-1]);//Sets the final position no matter what

        AutonPos[] result = new AutonPos[points.length];
        result[0] = new AutonPos(points[0], 0);

        //Finds the lengths of each segment
        for(int i = 1; i < result.length - 1; ){

            //straight length logic
            double length = Vector.subVectors(points[i+1], points[i]).r;
            result[i] = new AutonPos(points[i + 1], length);

            i++;

            //arc length logic
            if(i >= result.length - 1) break;

            //reversing the arc points to find the angle
            double vecDiff = Math.abs(Vector.subVectors(points[i + 1], points[i]).r);//absolute difference between the two vectors
            double arcAng = Math.asin((vecDiff / 2.0) / minR) * 2;

            result[i] = new AutonPos(points[i + 1], minR * arcAng);

            i++;
        }

        return result;
    }

    double totalDistStrafe = 0;
    double totalDistAng = 0;
    double totalDist = 0;
    AutonPos[] locations;
    double startTime;
    double decelTime;
    double maxVelTime;
    double accelTime;
    double accelDist;
    double decelDist;
    double maxVelDist;
    public void initialize(){
        startLoc = new Vector(r.sensors.odo.botLocation);
        startAngle = r.sensors.odo.botAngle;
        interpIdx = 0;
        locations = formulateArcs(MIN_R, startLoc, wayPoints);

        for(int i = 0; i < locations.length; i++){
            totalDistStrafe += locations[i].value;
        }
        totalDistAng = (startAngle - locations[locations.length-1].value) * MIN_R;

        totalDist = totalDistAng + totalDistStrafe;

        double distThreshold = (AutonCal.driveBase.maxVel * AutonCal.driveBase.maxVel) / AutonCal.driveBase.maxAccel;
       
        if (totalDist >= distThreshold){
            //3 step (Accel - constV - Decel)
            accelDist = distThreshold / 2;
            decelDist = distThreshold / 2;
            maxVelDist = totalDist - distThreshold;
            accelTime =  Math.sqrt(distThreshold / AutonCal.driveBase.maxAccel);
            maxVelTime = accelTime + maxVelDist / AutonCal.driveBase.maxVel;
            decelTime = maxVelTime + accelTime;
        } else{
            //2 step (Accel - Decel)
            maxVelTime = 0;
            maxVelDist = 0;
            accelDist = totalDist / 2;
            decelDist = totalDist / 2;
            accelTime =  Math.sqrt(totalDist / AutonCal.driveBase.maxAccel);
            decelTime = 2 * accelTime;
        }
        startTime = Timer.getFPGATimestamp();
    }

    int interpIdx = 0;
    //returns the theta of our drive vector based on which segment is being driven
    double interpWaypoints(double minR, Vector currPos, double expectedLoc, double totalDist, AutonPos[] locations){

        for(; interpIdx < locations.length - 2; interpIdx++){
            //find the line segment we should be at based on distance travelled
            if(locations[interpIdx].value <= (totalDist - expectedLoc) && (totalDist - expectedLoc) < locations[interpIdx+1].value) break;
        }
        
        double tgtAng;
        if(interpIdx % 2 == 0){//straight line indexes
            tgtAng = Vector.subVectors(locations[interpIdx+1].xy, locations[interpIdx].xy).theta;
        } else {//arc indexes
            double firstSegmentAng = Vector.subVectors(locations[interpIdx-1].xy, locations[interpIdx-2].xy).theta;//line segment before the arc
            double secondSegmentAng = Vector.subVectors(locations[interpIdx+1].xy, locations[interpIdx].xy).theta;//line segment after the arc

            double angleDiff;
            if(secondSegmentAng < firstSegmentAng){
                angleDiff = firstSegmentAng - Math.PI/2;
            } else {
                angleDiff = firstSegmentAng + Math.PI/2;
            }

            //center of arc being driven
            Vector centerOfRot = Vector.addVectors(locations[interpIdx].xy, new Vector(minR, angleDiff));

            if(secondSegmentAng < firstSegmentAng){
                tgtAng = Vector.subVectors(currPos, centerOfRot).theta - Math.PI/2;
            } else {
                tgtAng = Vector.subVectors(currPos, centerOfRot).theta + Math.PI/2;
            }
        }

        return tgtAng;
    }
    
    public void execute (){
        double runTime = Timer.getFPGATimestamp() - startTime;
        Vector currentPos = r.sensors.odo.botLocation;
        double currentAng = r.sensors.odo.botAngle;
        
        double targetPosR;
        double targetVelR;
        double targetAccel;
        if (runTime < accelTime){
            //stage 1
            targetAccel = AutonCal.driveBase.maxAccel;
            targetPosR = 0.5 * targetAccel * runTime * runTime;
            targetVelR = targetAccel * runTime;
        } else if (runTime < maxVelTime) { 
            //stage 2
            targetAccel = 0;
            targetVelR = AutonCal.driveBase.maxVel;
            targetPosR = ((runTime - accelTime) * targetVelR) + accelDist;
        } else if (runTime < decelTime) { 
            //stage 3
            double t = runTime - decelTime;
            targetAccel = -AutonCal.driveBase.maxAccel;
            targetPosR = 0.5 * targetAccel * t * t + totalDist;
            targetVelR = t * targetAccel;
        } else { 
            //end
            targetPosR = totalDist;
            targetVelR = 0;
            targetAccel = 0;
        }


        //strafe
        double strafeRatio = totalDistStrafe / totalDist;

        double strafeRPos = targetPosR * strafeRatio;
        double strafeRVel = targetVelR * strafeRatio;
        double strafeRAcc = targetAccel * strafeRatio;

        double strafeAng = interpWaypoints(MIN_R, currentPos, strafeRPos, totalDist, locations);
        Vector targetPosStrafe = new Vector(strafeRPos, strafeAng);
        Vector targetVelStrafe = new Vector(strafeRVel, strafeAng);

        //PID strafe err correction
        targetPosStrafe.sub(currentPos);//error value
        targetPosStrafe.r *= mpCals.kP_MP;
        targetVelStrafe.add(targetPosStrafe);


        //angle
        double angRatio = totalDistAng / totalDist;

        double angleRPos = targetPosR * angRatio;
        double angleRVel = targetVelR * angRatio;
        double angleRAcc = targetAccel * angRatio;
        
        //PID angle err correction
        double angleError = angleRPos - currentAng;
        angleError *= mpCals.kP_MP;
        angleRVel += angleError;


        //output
        Vector drivePwr = new Vector(targetVelStrafe);
        drivePwr.r = (mpCals.kA * strafeRAcc) + (mpCals.kV * targetVelStrafe.r) + mpCals.kS;

        double anglePwr = (mpCals.kA * angleRAcc) + (mpCals.kV * angleRVel) + mpCals.kS;

        //voltage compensation
        double volts = r.lights.pdh.getVoltage();
        if(volts < 5 || volts > 15){
            volts = 12;
        }
        drivePwr.r *= 12.0 / volts;
        anglePwr *= 12.0 / volts;

        r.driveTrain.swerveMP(drivePwr, anglePwr);
    }

    public boolean isFinished(){
        return false;
    }

    public void end(){

    }
}
