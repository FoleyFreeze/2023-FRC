package frc.robot.commands.Auton.AdvancedMovement;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.ListIterator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Auton.AutonCal.MPCals;
import frc.robot.util.Angle;
import frc.robot.util.Vector;

public class FancyMotionProfile extends CommandBase {
    
    RobotContainer r;
    MPCals cals;

    ArrayList<Vector> waypoints;
    ArrayList<Tag> tags;

    final double maxWheelSpeed = Math.PI/2.0 / 0.2 * 0.9; //90 deg in 0.2 seconds, but leave some margin
    final double botRadius;
    final boolean allowArcAccel = false; //allow velocity to change on arc
    final boolean allowArcAngle = false; //allow angle to change on arc

    Vector globalOffset; //vision updates this

    LinkedList<Step> path;
    LinkedList<Tag> pathTags;
    Vector startLoc;
    double flag;
    double startAng;

    public FancyMotionProfile(RobotContainer r, MPCals cals, ArrayList<Vector> waypoints, ArrayList<Tag> tags){
        this.r = r;
        this.cals = cals;

        this.waypoints = waypoints;
        this.tags = tags;

        botRadius = r.driveTrain.cals.wheelCals[0].wheelLocation.r;
    }

    public void initialize(){
        startLoc = new Vector(r.sensors.odo.botLocation);
        startAng = r.sensors.odo.botAngle;
        path = generatePath(startLoc, startAng, waypoints);
        pathTags = addTags(path, tags);
        calculateTimes(path);
        globalOffset = new Vector(0,0);
    }

    public void execute(){

    }

    public boolean isFinished(){
        return false;
    }

    public void end(boolean interrupted){

    }


    //local classes
    

    private class Step{
        boolean isArc;
        Vector endPos;
        double endVel;
        double velLimit;
        double endAngle;
        double endAccelTime;
        double endCvTime;
        double endDecelTime;
        double dist;
        double length;//includes angle change
    }

    private LinkedList<Step> generatePath(Vector startLoc, double startAng, ArrayList<Vector> waypoints){
        LinkedList<Step> steps = new LinkedList<>();
        Step s = new Step();
        s.endAngle = startAng;
        s.endPos = startLoc;
        s.velLimit = cals.maxVel;
        steps.add(s);

        Step prevStep;

        for(int waypointIdx=0; waypointIdx<waypoints.size(); waypointIdx++){

            Vector waypoint = waypoints.get(waypointIdx);

            //construct next step(s)
            prevStep = steps.getLast();
            if(waypointIdx < waypoints.size()-1){
                //multiple waypoints left, connect with arc
                Vector nextWaypoint = waypoints.get(waypointIdx+1);
                Vector atob = Vector.subVectors(waypoint, prevStep.endPos);
                Vector btoc = Vector.subVectors(nextWaypoint, waypoint);

                double arcAngle = Math.acos(atob.dot(btoc)/atob.r/btoc.r);
                double minR = cals.maxVel / maxWheelSpeed;
                double dist = minR * Math.tan(arcAngle / 2.0);

                Vector arcStart = Vector.subVectors(waypoint, new Vector(dist, atob.theta));
                Vector arcEnd = Vector.subVectors(nextWaypoint, new Vector(btoc.r - dist, btoc.theta));

                Vector circleCenter = Vector.subVectors(waypoint, arcStart);
                circleCenter.theta += Math.PI/2;
                circleCenter.r = minR;
                circleCenter.add(arcStart);

                //drive to arcStart
                s = new Step();
                s.endPos = arcStart;
                s.endAngle = prevStep.endAngle;
                s.endVel = cals.maxVel;
                s.length = Vector.subVectors(arcStart, prevStep.endPos).r;
                s.dist = s.length;
                s.velLimit = cals.maxVel;
                steps.add(s);

                //drive the arc
                s = new Step();
                s.isArc = true;
                s.endPos = arcEnd;
                s.endAngle = prevStep.endAngle;
                s.endVel = cals.maxVel;
                s.length = minR * arcAngle;
                s.dist = s.length;
                s.velLimit = cals.maxVel;
                steps.add(s);

            } else {
                //last waypoint, end the path
                s = new Step();
                s.endPos = waypoint;
                s.endAngle = prevStep.endAngle;
                s.endVel = 0;
                s.length = Vector.subVectors(waypoint, prevStep.endPos).r;
                s.dist = s.length;
                s.velLimit = cals.maxVel;
                steps.add(s);
            }

        }

        return steps;
    }

    //tags are added by adding new steps with angle targets or velocity restrictions
    //later scheduled tags (flags/vision) are put in their own list to be called during execute as they are reached
    private LinkedList<Tag> addTags(LinkedList<Step> path, ArrayList<Tag> tags){
        LinkedList<Tag> tagList = new LinkedList<>();

        ArrayList<Tag> vList = new ArrayList<>();
        
        ListIterator<Step> stepIter = path.listIterator();
        Step prevStep = stepIter.next();//this is the start position
        Step currStep = stepIter.next();//this is the first drive
        int stepIdx = 1;//the index of currStep
        int tagStep = 0;//the non-arc or non-start-step the tag belongs to, effectively the waypoint index
        Step angleStep = prevStep; //the last step with a forced angle
        Step velStep = prevStep; //the last step with a forced velocity
        double currentFraction = 0;//the tag step fraction already covered by previous tag steps

        double maxVel = cals.maxVel;

        ArrayList<Step> backStepList = new ArrayList<>();

        for(Tag tag : tags) {
            double targetIndex = tag.step - tagStep;
            while(targetIndex > 1){
                currentFraction = 0;
                if(stepIter.hasNext()){
                    prevStep = currStep;
                    currStep = stepIter.next();
                    stepIdx++;
                    tagStep++;
                }
                
                if(currStep.isArc) {
                    if(!stepIter.hasNext()) {
                        System.out.println("ERROR: Tag: " + tag.step + " happens after the last step");
                        break;
                    }
                    prevStep = currStep;
                    currStep = stepIter.next();
                    stepIdx++;
                }

                targetIndex = tag.step - tagStep;
            }
            targetIndex = (tag.step - tagStep - currentFraction) / (1-currentFraction);

            //angle tags modify the path
            if(tag.type == Tag.Type.ANGLE){
                double stepDist = currStep.length * targetIndex;
                Step s = new Step();
                s.endPos = Vector.subVectors(currStep.endPos, prevStep.endPos);
                s.endPos.r = stepDist;
                s.endPos.add(prevStep.endPos);
                s.length = stepDist;
                s.dist = s.length;
                s.velLimit = cals.maxVel;
                currStep.length -= stepDist;
                currStep.dist -= stepDist;
                currStep.endAngle = tag.value;
                s.endAngle = tag.value;
                s.endVel = maxVel;
                
                //modify all steps between currStep and angleStep to perform the rotation
                backStepList.add(s);
                double totalLength = s.length;
                stepIter.previous();//this returns currStep
                Step step = stepIter.previous();
                while(!step.equals(angleStep)){
                    if(!step.isArc || allowArcAngle){
                        //dont turn during arc steps, but still put them in the backlog so their angles can be updated
                        totalLength += step.length;
                    }
                    backStepList.add(step);
                    step = stepIter.previous();
                }
                double anglePerLen = Angle.normRad(tag.value - angleStep.endAngle) / totalLength;
                double startAngle = s.endAngle;
                double currAngle = startAngle;
                double angleDist = 0;
                //note this loops backwards (first index is last step)
                for(Step backStep : backStepList){
                    if(!backStep.isArc || allowArcAngle){
                        //startAngle -= anglePerLen*angleDist;
                        angleDist += backStep.length;
                        backStep.length += Math.abs(anglePerLen*backStep.length*botRadius);
                    }
                    //still update the angle on arc steps
                    backStep.endAngle = currAngle;
                    currAngle = startAngle - anglePerLen*angleDist;
                }
                //get iter back to the current step
                while(stepIter.next() != currStep);
                stepIter.previous();
                //add s before currStep
                stepIter.add(s);
                stepIter.next();
                stepIdx++;//increment for s

                //get ready for next tag
                backStepList.clear();
                angleStep = s;
                prevStep = s;
                currentFraction = targetIndex;

            } else {
                if(tag.type == Tag.Type.VEL){
                    Tag t = new Tag();
                    t.type = tag.type;
                    t.value = tag.value;
                    t.step = stepIdx + targetIndex;
                    vList.add(t);
                } else {
                    //add tag to tags list so it runs during execute
                    Tag t = new Tag();
                    t.type = tag.type;
                    t.value = tag.value;
                    //this math will need to change if we are ok with vision/flags on a curve
                    t.step = stepIdx + targetIndex;
                    tagList.add(t);
                }
            }
        }


        //loop through the velocity tags and calculate start/end velocities for each step
        stepIter = path.listIterator();
        ListIterator<Tag> vTagIter = vList.listIterator();
        Tag currTag = null;
        double targetIndex = 0;
        if(vTagIter.hasNext()) {
            currTag = vTagIter.next();
            targetIndex = currTag.step - stepIdx;
        }

        currStep = stepIter.next();
        stepIdx = 0;
        while(stepIter.hasNext()){
            prevStep = currStep;
            currStep = stepIter.next();
            stepIdx++;
            
            //add new step for the velocity change
            if(currTag != null && targetIndex >= 0 && targetIndex < 1){
                double stepDist = currStep.dist * targetIndex;
                Step s = new Step();
                s.endPos = Vector.subVectors(currStep.endPos, prevStep.endPos);
                s.endPos.r = stepDist;
                s.endPos.add(prevStep.endPos);
                double deltaAngle = Angle.normRad(currStep.endAngle - prevStep.endAngle);
                double newAngle = prevStep.endAngle + deltaAngle * (stepDist/currStep.dist);
                s.endAngle = newAngle;
                s.length = stepDist;
                s.dist = s.length;
                currStep.dist -= stepDist;
                double deltaLength = newAngle * botRadius;
                s.length += deltaLength;
                currStep.length -= stepDist + deltaLength;
                
                if(currTag.value > maxVel){
                    //accel, no need to do anything fancy

                    s.endVel = Math.sqrt(prevStep.endVel*prevStep.endVel + 2*cals.maxAccel*s.length);
                    s.endVel = Math.min(maxVel,s.endVel);//this step is still limited to the old vel limit
                    s.velLimit = currStep.velLimit;

                    maxVel = currTag.value;
                    maxVel = Math.min(maxVel, cals.maxVel);

                    currStep.endVel = Math.sqrt(s.endVel*s.endVel + 2*cals.maxAccel*currStep.length);
                    currStep.endVel = Math.min(maxVel, currStep.endVel);
                } else {
                    //decel, look back to make sure we can slow down
                    maxVel = currTag.value;
                    currStep.endVel = maxVel;
                    
                    s.endVel = maxVel;
                    Step secondStep = s;
                    Step firstStep = stepIter.previous();
                    //back calculate how fast step 2 can start if it wants to end at the new velocity
                    double startVel = Math.sqrt(secondStep.endVel*secondStep.endVel + 2*cals.maxAccel*secondStep.length);
                    while(startVel < firstStep.endVel) {
                        //if it must start slower than it can, the the previous step also needs to slow down
                        firstStep.endVel = startVel;
                        secondStep = firstStep;
                        firstStep = stepIter.previous();
                        startVel = Math.sqrt(secondStep.endVel*secondStep.endVel + 2*cals.maxAccel*secondStep.length);
                    }

                    //get iter back to currStep
                    while(!stepIter.next().equals(currStep));
                }

                //go back a step to add s before currStep
                stepIter.previous();
                stepIter.add(s);
                stepIter.next();
                incrementTags(tagList, stepIdx, targetIndex);
                stepIdx++;//increment for s
                prevStep = s;

                if(vTagIter.hasNext()) currTag = vTagIter.next();
                targetIndex = currTag.step - stepIdx;
            }

            //assume accel until proven otherwise
            currStep.endVel = Math.sqrt(prevStep.endVel*prevStep.endVel + 2*cals.maxAccel*currStep.length);
            currStep.endVel = Math.min(maxVel, currStep.endVel);
            currStep.velLimit = maxVel;
        }

        //decel to zero for the last step
        currStep.endVel = 0;
        Step secondStep = currStep;
        stepIter.previous();//this returns currentStep
        Step firstStep = stepIter.previous();
        //back calculate how fast step 2 can start if it wants to end at the new velocity
        double startVel = Math.sqrt(secondStep.endVel*secondStep.endVel + 2*cals.maxAccel*secondStep.length);
        while(startVel < firstStep.endVel) {
            //if it must start slower than it can, the the previous step also needs to slow down
            firstStep.endVel = startVel;
            secondStep = firstStep;
            firstStep = stepIter.previous();
            startVel = Math.sqrt(secondStep.endVel*secondStep.endVel + 2*cals.maxAccel*secondStep.length);
        }

        
            
        return tagList;
    }

    private void incrementTags(LinkedList<Tag> tagList, int step, double split){
        for(Tag t : tagList){
            if(t.step > step){
                if(t.step < step+1){
                    //on the split step
                    double frac = t.step-step;
                    if(frac > split){
                        double newfrac = (frac - split) / (1-split);
                        t.step = step + 1 + newfrac;
                    } else {
                        double newfrac = frac / split;
                        t.step = step + newfrac;
                    }
                } else {
                    //after this step
                    t.step++;
                }
            }
        }
    }

    private void calculateTimes(LinkedList<Step> path){
        double prevVel = 0;
        double prevEndTime = 0;
        for(Step s: path){
            if(s.length == 0){//start step
                s.endAccelTime = prevEndTime;
                s.endCvTime = prevEndTime;
                s.endDecelTime = prevEndTime;
            } else if(prevVel == s.endVel && s.endVel == s.velLimit){//constant velocity only
                s.endAccelTime = prevEndTime;
                s.endCvTime = prevEndTime + (s.length / s.velLimit);
                s.endDecelTime = s.endCvTime;
            } else if(prevVel == s.velLimit){//constant velocity to deceleration
                double t3 = (prevVel - s.endVel) / cals.maxAccel;
                double x3 = prevVel * t3 - 0.5 * cals.maxAccel * (t3 * t3);
                double t2 = (s.length - x3) / prevVel;
                s.endAccelTime = prevEndTime;
                s.endCvTime = prevEndTime + t2;
                s.endDecelTime = prevEndTime + t2 + t3;
            } else if(s.endVel == s.velLimit){//acceleration to constant velocity
                double t1 = (s.endVel - prevVel) / cals.maxAccel;
                double x1 = prevVel * t1 + 0.5 * cals.maxAccel * (t1 * t1);
                double t2 = (s.length - x1) / s.endVel;
                s.endAccelTime = prevEndTime + t1;
                s.endCvTime = prevEndTime + t1 + t2;
                s.endDecelTime = s.endCvTime;
            } else if(valuesAreClose(s.endVel, Math.sqrt(prevVel * prevVel + 2 * cals.maxAccel * s.length), 0.01)){//acceleration only
                double t = (s.endVel - prevVel) / cals.maxAccel;
                s.endAccelTime = prevEndTime + t;
                s.endCvTime = s.endAccelTime;
                s.endDecelTime = s.endAccelTime;
            } else if(valuesAreClose(prevVel, Math.sqrt(s.endVel * s.endVel + 2 * cals.maxAccel * s.length), 0.01)){//deceleration only
                double t = (prevVel - s.endVel) / cals.maxAccel;
                s.endAccelTime = prevEndTime;
                s.endCvTime = s.endAccelTime;
                s.endDecelTime = s.endAccelTime + t;
            } else {
                double velLimSq = s.velLimit * s.velLimit;
                double velInitSq = prevVel * prevVel;
                double velFinSq = s.endVel * s.endVel;
                double accelDist = (velLimSq - velInitSq) / (2 * cals.maxAccel);
                double decelDist = (velLimSq - velFinSq) / (2 * cals.maxAccel);
                if(s.length < accelDist + decelDist){//full 2-step, not reaching constant v
                    double topV = 0;//TODO: This is wrong
                    double t1 = (topV - prevVel) / cals.maxAccel;
                    double t2 = (topV - s.endVel) / cals.maxAccel;
                    s.endAccelTime = prevEndTime + t1;
                    s.endCvTime = s.endAccelTime;
                    s.endDecelTime = s.endAccelTime + t2;
                } else {//full 3-step, reaches constant v
                    double t1 = (s.velLimit - prevVel) / cals.maxAccel;
                    double t2 = (s.length - accelDist - decelDist) / s.velLimit;
                    double t3 = (s.velLimit - s.endVel) / cals.maxAccel;
                    s.endAccelTime = prevEndTime + t1;
                    s.endCvTime = s.endAccelTime + t2;
                    s.endDecelTime = s.endCvTime + t3;
                }
            }

            prevEndTime = s.endDecelTime;
            prevVel = s.endVel;
        }
    }

    private boolean valuesAreClose(double v1, double v2, double maxErr){
        return Math.abs(v1 - v2) < maxErr;
    }
}
