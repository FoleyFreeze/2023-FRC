package frc.robot.commands.Auton.AdvancedMovement;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.ListIterator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    final boolean debug = true;

    Vector globalOffset; //vision updates this

    LinkedList<Step> path;
    LinkedList<Tag> pathTags;
    Vector startLoc;
    public double flag;
    double visionState;
    double startAng;

    public FancyMotionProfile(RobotContainer r, MPCals cals, ArrayList<Vector> waypoints, ArrayList<Tag> tags){
        this.r = r;
        this.cals = cals;

        this.waypoints = waypoints;
        this.tags = tags;

        botRadius = r.driveTrain.cals.wheelCals[0].wheelLocation.r;
    }

    double startTime;

    public void initialize(){
        startLoc = new Vector(r.sensors.odo.botLocation);
        startAng = r.sensors.odo.botAngle;
        path = generatePath(startLoc, startAng, waypoints);
        pathTags = addTags(path, tags);
        tagIter = pathTags.listIterator();
        currTag = null;
        calculateTimes(path);
        globalOffset = new Vector(0,0);
        flag = 0;
        visionState = 0;

        startTime = Timer.getFPGATimestamp();
    }

    public void execute(double t){
        if(complete) return;

        checkTags();
        runVision();

        //get motion profile
        getAVP(t);

        //global offset provides a offset to the entire path based on vision
        Vector currentPos = Vector.addVectors(r.sensors.odo.botLocation, globalOffset);

        //PID
        Vector errVec = Vector.subVectors(targetPosVec, currentPos);
        double errorMag = errVec.r;
        errVec.r *= cals.kP_MP;
        Vector totalVel = new Vector(targetVelVec).add(errVec);
        double velocity = totalVel.r;


        double currentAngle = r.sensors.odo.botAngle;
        double angleError = Angle.normRad(targetAngle - currentAngle);
        double totalAngleVel = angleError * botRadius * cals.kP_MP + targetAngleVel;
        double anglePower = targetAngleAccel*cals.kA + totalAngleVel*cals.kV + cals.kS;

        //output
        Vector powerVec = new Vector(targetAccel, targetVelVec.theta);
        totalVel.r *= cals.kV;
        powerVec.r *= cals.kA;
        powerVec.add(totalVel);
        powerVec.r += cals.kS;

        //rotate the strafe vector by the angular velocity (in 20ms) as a feedforward
        //see https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
        //basically we are offsetting for the discretization of our model (due to not modeling module velocities and accelerations)
        double rotationFeedForward = totalAngleVel * 0.02 / botRadius;
        powerVec.theta -= rotationFeedForward;

        if(debug) System.out.format("s:%d, t:%.2f, p:%.2f, pA:%.2f err:%.0f, v:%.0f, a:%.0f, eA:%.0f, pA:%.0f, vA:%.0f, aA:%.0f, ffdA:%.1f, t1:%.1f, t2:%.1f, t3:%.1f\n", 
                stepIdx,t,powerVec.r,anglePower,errorMag,velocity,targetAccel,
                Math.toDegrees(angleError),Math.toDegrees(targetAngle),totalAngleVel,targetAngleAccel,Math.toDegrees(-rotationFeedForward),
                currStep.endAccelTime,currStep.endCvTime,currStep.endDecelTime);

        //voltage compensation
        double volts = r.lights.pdh.getVoltage();
        if(volts < 5 || volts > 15){
            volts = 12;
        }
        powerVec.r *= 12.0 / volts;
        r.driveTrain.swerveMPAng(powerVec,anglePower);
    }

    public void execute(){
        double runTime = Timer.getFPGATimestamp() - startTime;
        execute(runTime);
    }

    public boolean isFinished(){
        return complete;
    }

    public void end(boolean interrupted){
        r.driveTrain.swerveMPAng(new Vector(0,0),0);
    }


    //local classes
    

    private class Step{
        boolean isArc;
        Vector arcCenter;
        double arcLen;
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
                double axb = atob.cross2D(btoc);
                if(axb > 0){
                    circleCenter.theta += Math.PI/2;
                } else {
                    circleCenter.theta -= Math.PI/2;
                }
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
                if(axb > 0){
                    s.arcLen = arcAngle;    
                } else {
                    s.arcLen = -arcAngle;
                }
                s.arcCenter = circleCenter;
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

    private Vector targetPosVec;
    private Vector targetVelVec;
    private double targetAccel;
    private double targetAngle;
    private double targetAngleVel;
    private double targetAngleAccel;
    Step prevStep;
    Step currStep;
    int stepIdx;
    double frac;
    boolean complete = false;
    private void getAVP(double t){
        ListIterator<Step> iter = path.listIterator();
        prevStep = iter.next();
        currStep = iter.next();
        stepIdx = 1;
        complete = false;
        while(iter.hasNext() && currStep.endDecelTime < t){
            prevStep = currStep;
            currStep = iter.next();
            stepIdx++;
        }


        double targetTotalAccel;
        double targetTotalPos;
        double targetTotalVel;
        int stage;

        if (t < currStep.endAccelTime){ 
            //stage 1
            stage = 1;
            double t1 = t - prevStep.endDecelTime;
            targetTotalAccel = cals.maxAccel;
            targetTotalPos = prevStep.endVel*t1 + 0.5*targetTotalAccel*t1*t1;
            targetTotalVel = prevStep.endVel + targetTotalAccel*t1;
        } else if (t < currStep.endCvTime) { 
            //stage 2
            stage = 2;
            double t1 = currStep.endAccelTime - prevStep.endDecelTime;
            double t2 = t - currStep.endAccelTime;
            double x1 = prevStep.endVel*t1 + 0.5*cals.maxAccel*t1*t1;
            targetTotalAccel = 0;
            targetTotalVel = currStep.velLimit;
            targetTotalPos = x1 + targetTotalVel*t2;
        } else if (t < currStep.endDecelTime) { 
            //stage 3
            stage = 3;
            double t1 = currStep.endAccelTime - prevStep.endDecelTime;
            double t2 = currStep.endCvTime - currStep.endAccelTime;
            double t3 = t - currStep.endCvTime;
            double x1 = prevStep.endVel * t1 + 0.5*cals.maxAccel*t1*t1;
            double v2 = prevStep.endVel + t1*cals.maxAccel;
            double x2 = v2*t2;
            targetTotalAccel = -cals.maxAccel;
            targetTotalPos = v2*t3 + 0.5*targetTotalAccel*t3*t3 + x1 + x2;
            targetTotalVel = v2 + targetTotalAccel*t3;
        } else {
            //end
            stage = 4;
            complete = true;
            targetTotalPos = currStep.length;
            targetTotalVel = 0;
            targetTotalAccel = 0;
        }

        //determine split between dist and angle
        frac = targetTotalPos / currStep.length;
        double distFrac = currStep.dist / currStep.length;
        double posFrac = targetTotalPos * distFrac;

        targetAccel = targetTotalAccel * distFrac;
        targetAngleVel = targetTotalVel * (1-distFrac);
        targetAngleAccel = targetTotalAccel * (1-distFrac);

        if(currStep.isArc){
            double arcAngle = currStep.arcLen * frac;
            
            targetPosVec = Vector.subVectors(prevStep.endPos, currStep.arcCenter);
            targetPosVec.theta += arcAngle;
            
            //for target velocity we will want to look ahead at least 1 timestep (maybe more? whats the swerve angle lag?)
            double lookAheadFrac = 0.02 / (currStep.dist / (targetTotalVel*posFrac)); //what fraction of this arc is covered in the next 20ms
            targetVelVec = new Vector(targetPosVec);
            targetVelVec.theta += currStep.arcLen * lookAheadFrac;
            targetVelVec.theta -= Math.signum(currStep.arcLen) * Math.PI/2;
            targetVelVec.r = targetTotalVel * posFrac;

            targetPosVec.add(currStep.arcCenter);
        } else {
            targetPosVec = Vector.subVectors(currStep.endPos, prevStep.endPos);
            targetPosVec.r = posFrac;
            targetVelVec = new Vector(targetTotalVel*distFrac,targetPosVec.theta);
            targetPosVec.add(prevStep.endPos); //target field location
        }

        double totalAngleDelta = Angle.normRad(currStep.endAngle - prevStep.endAngle);
        targetAngle = totalAngleDelta * frac + prevStep.endAngle; 

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
                    //determine distance required to equalize start/end speed
                    //then see how fast we can get to in half the remaining distance
                    double tEqual, xEqual, vEqual, startVel;
                    if(s.endVel > prevVel){
                        startVel = s.endVel;
                        vEqual = s.endVel - prevVel;
                        xEqual = (s.endVel*s.endVel - prevVel*prevVel) / (2.0 * cals.maxAccel);
                    } else {
                        startVel = prevVel;
                        vEqual = prevVel - s.endVel;
                        xEqual = (prevVel*prevVel - s.endVel*s.endVel) / (2.0 * cals.maxAccel);
                    }
                    tEqual = vEqual / cals.maxAccel;
                    double xRem = (s.length - xEqual)/2.0;
                    double maxVel = Math.sqrt(startVel*startVel + 2 * cals.maxAccel * xRem);
                    double t1 = (maxVel - startVel) / cals.maxAccel;
                    if(s.endVel > prevVel){
                        s.endAccelTime = prevEndTime + t1 + tEqual;
                        s.endCvTime = s.endAccelTime;
                        s.endDecelTime = s.endCvTime + t1;
                    } else {
                        s.endAccelTime = prevEndTime + t1;
                        s.endCvTime = s.endAccelTime;
                        s.endDecelTime = s.endCvTime + t1 + tEqual;
                    }

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

    double visionFilter = 0.5; //weight each image within the threshold this much
    double visionThresh = 24; //ignore any image that moves the global offset by more than this
    private void runVision(){
        //vision states:
        //  -1 : cubes
        //   1 : red side bump
        //   3 : red side sub
        //   6 : blue side sub
        //   8 : blue side bump
        //Note: this function will flip the selected tag based on reported driverstation color

        if(visionState == 0) return;

        Vector offset = null;
        if(visionState > 0){
            //april tags
            int targetTag = (int) visionState;
            if(DriverStation.getAlliance() == Alliance.Red){
                if(visionState > 5) targetTag = 9 - targetTag; 
            } else { //else blue
                if(visionState < 4) targetTag = 9 - targetTag;
            }

            offset = r.vision.getTagForAuton(targetTag);

        } else if(visionState == -1) {
            //cube

            //TODO: raw cube position, but needs to get converted to an offset
            //need to build an offset table using the negative numbers, -1 - -4 for the 4 positions
            offset = r.vision.getCubeVector(); 
        }

        if(offset == null) return;
        if(offset.r > visionThresh) return;
        
        Vector delta = Vector.subVectors(offset, globalOffset);
        delta.r *= visionFilter;
        globalOffset.add(delta);
        if(debug) System.out.println("VisonOffset is now: " + globalOffset.toStringXY());

    }

    ListIterator<Tag> tagIter;
    Tag currTag;
    private void checkTags(){
        //determines if the next flag tag is active
        if(currTag == null){
            if(!tagIter.hasNext()) return;
            currTag = tagIter.next();
        }

        while(currTag.step < stepIdx || Math.floor(currTag.step) == stepIdx && frac > currTag.step - stepIdx){
            //activate tag
            switch(currTag.type){
                case FLAG:
                    flag = currTag.value;
                    if(debug) System.out.println("Flag set to: " + flag);
                break;

                case VISION:
                    visionState = currTag.value;
                    if(debug) System.out.println("Vision set to: " + visionState);
                break;

                default:
                    System.out.println("Error, bad tag: " + currTag.type.name());
            }

            if(tagIter.hasNext()){
                currTag = tagIter.next();
            } else {
                currTag = null;
                return;
            }
        }
    }
}
