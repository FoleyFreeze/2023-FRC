package frc.robot.commands.Auton.AdvancedMovement;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal;
import frc.robot.commands.Auton.AutonPos;
import frc.robot.util.Vector;

public class MultiDimensionalMotionProfile extends CommandBase {

    public RobotContainer r;
    Vector startLoc;
    double startAngle;

    AutonPos[] wayPoints;

    final double MIN_R;

    public MultiDimensionalMotionProfile(RobotContainer r, AutonPos[] wayPoints){
        this.r = r;

        this.wayPoints = wayPoints;

        MIN_R = r.driveTrain.cals.wheelCals[0].wheelLocation.r;
        addRequirements(r.driveTrain);
    }

    public AutonPos[] formulateArcs(double minR, Vector startPos, AutonPos... wayPoints){
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

            Vector firstVec = Vector.subVector(midPoint, firstPoint);
            Vector lastVec = Vector.subVector(lastPoint, midPoint);

            //adds the complementary angles
            double angle = (Math.PI / 2 - firstVec.theta) + (Math.PI / 2 - lastVec.theta);

            double dist;
            if(angle != Math.PI){
                dist = minR / Math.tan(angle / 2);
            } else {
                dist = 0;
            }

            points[i+1] = Vector.subVector(midPoint, new Vector(dist, firstVec.theta));
            points[i+2] = Vector.subVector(lastPoint, new Vector(lastVec.r - dist, lastVec.theta));
        }
        
        points[points.length-1] = new Vector(pointsWStart[pointsWStart.length-1]);//Sets the final position no matter what

        AutonPos[] result = new AutonPos[points.length];
        result[0] = new AutonPos(points[0], 0);

        //Finds the lengths of each segment
        for(int i = 1; i < result.length - 1; ){

            //straight length logic
            double length = Vector.subVector(points[i+1], points[i]).r;
            result[i] = new AutonPos(points[i + 1], length);

            i++;

            //arc length logic
            if(i >= result.length - 1) break;

            //reversing the arc points to find the angle
            double vecDiff = Math.abs(Vector.subVector(points[i + 1], points[i]).r);//absolute difference between the two vectors
            double arcAng = Math.asin((vecDiff / 2.0) / minR) * 2;

            result[i] = new AutonPos(points[i + 1], minR * arcAng);

            i++;
        }

        return result;
    }

    //the distances are individual
    //but the times are cumulative
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
        interpIdx = 0;
        locations = formulateArcs(MIN_R, startLoc, wayPoints);

        for(int i = 0; i < locations.length; i++){
            totalDist += locations[i].value;
        }

        double distThreshold = (AutonCal.maxVel * AutonCal.maxVel) / AutonCal.maxAccel;
       
        if (totalDist >= distThreshold){
            //3 step (Accel - constV - Decel)
            accelDist = distThreshold / 2;
            decelDist = distThreshold / 2;
            maxVelDist = totalDist - distThreshold;
            accelTime =  Math.sqrt(distThreshold / AutonCal.maxAccel);
            maxVelTime = accelTime + maxVelDist / AutonCal.maxVel;
            decelTime = maxVelTime + accelTime;
        } else{
            //2 step (Accel - Decel)
            maxVelTime = 0;
            maxVelDist = 0;
            accelDist = totalDist / 2;
            decelDist = totalDist / 2;
            accelTime =  Math.sqrt(totalDist / AutonCal.maxAccel);
            decelTime = 2 * accelTime;
        }
        startTime = Timer.getFPGATimestamp();
    }

    int interpIdx = 0;
    //returns the theta of our drive vector
    double interpWaypoints(double minR, Vector currPos, double travelledDist, double totalDist, AutonPos[] locations){

        for(; interpIdx < locations.length - 2; interpIdx++){
            if(locations[interpIdx].value <= (totalDist - travelledDist) && (totalDist - travelledDist) < locations[interpIdx+1].value) break;
        }
        
        double tgtAng;
        if(interpIdx % 2 == 0){//straight line indexes
            tgtAng = Vector.subVector(locations[interpIdx+1].xy, locations[interpIdx].xy).theta;
        } else {//arc indexes
            double firstSegmentAng = Vector.subVector(locations[interpIdx-1].xy, locations[interpIdx-2].xy).theta;//line segment before the arc
            double secondSegmentAng = Vector.subVector(locations[interpIdx+1].xy, locations[interpIdx].xy).theta;//line segment after the arc

            double angleDiff;
            if(secondSegmentAng < firstSegmentAng){
                angleDiff = firstSegmentAng - Math.PI/2;
            } else {
                angleDiff = firstSegmentAng + Math.PI/2;
            }

            Vector centerOfRot = Vector.addVectors(locations[interpIdx].xy, new Vector(minR, angleDiff));

            if(secondSegmentAng < firstSegmentAng){
                tgtAng = Vector.subVector(currPos, centerOfRot).theta - Math.PI/2;
            } else {
                tgtAng = Vector.subVector(currPos, centerOfRot).theta + Math.PI/2;
            }
        }

        return tgtAng;
    }

    double interpBotAng(double travelledDist, double totalDist){

    }
    
    public void execute (){
        double runTime = Timer.getFPGATimestamp() - startTime;
        
        Vector targetPos = new Vector(0,totalDistance.theta);
        double targetAng;
        Vector targetVel = new Vector(0,totalDistance.theta);
        double targetAccel;
        if (runTime < accelTime){ 
            //stage 1
            targetAccel = AutonCal.maxAccel;
            targetPos.r = 0.5 * targetAccel * runTime * runTime;
            targetVel.r = targetAccel * runTime;
        } else if (runTime < maxVelTime) { 
            //stage 2
            targetAccel = 0;
            targetVel.r = AutonCal.maxVel;
            targetPos.r = ((runTime - accelTime) * targetVel.r) + accelDist;
        } else if (runTime < decelTime) { 
            //stage 3
            double t = runTime - decelTime;
            targetAccel = -AutonCal.maxAccel;
            targetPos.r = 0.5 * targetAccel * t * t + totalDist;
            targetVel.r = t * targetAccel;
        } else { 
            //end
            targetPos.r = totalDist;
            targetVel.r = 0;
            targetAccel = 0;
        }
        
        //PID
        Vector currentPos = r.sensors.odo.botLocation;
        targetPos.add(startLoc).negate().add(currentPos).negate();
        targetPos.r *= AutonCal.kP_MP;
        targetVel.add(targetPos);


        //output
        r.driveTrain.swerveMP(targetVel, targetAccel);
    }

    public boolean isFinished(){
        return false;
    }

    public void end(){

    }
}
