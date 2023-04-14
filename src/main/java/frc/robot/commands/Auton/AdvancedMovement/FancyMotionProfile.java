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

    final double maxWheelSpeed = Math.PI/2.0 / 0.2 * 0.8; //90 deg in 0.2 seconds, but leave some margin
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
    public enum Type {
        ANGLE,VEL,FLAG,VISION
    }

    public class Tag{
        double step;
        Type type;
        double value;
    }

    public class Step{
        boolean isArc;
        Vector endPos;
        double endVel;
        double endAngle;
        double endTime;
        double dist;
        double length;//includes angle change
    }

    private LinkedList<Step> generatePath(Vector startLoc, double startAng, ArrayList<Vector> waypoints){
        LinkedList<Step> steps = new LinkedList<>();
        Step s = new Step();
        s.endAngle = startAng;
        s.endPos = startLoc;
        steps.add(s);

        Step prevStep;

        for(int waypointIdx=0; waypointIdx<waypoints.size(); waypointIdx++){

            Vector waypoint = waypoints.get(waypointIdx);

            //construct next step(s)
            prevStep = steps.get(steps.size()-1);
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

                //drive to arcStart
                s = new Step();
                s.endPos = arcStart;
                s.endAngle = prevStep.endAngle;
                s.endVel = cals.maxVel;
                s.length = Vector.subVectors(arcStart, waypoint).r;
                s.dist = s.length;
                steps.add(s);

                //drive the arc
                s = new Step();
                s.isArc = true;
                s.endPos = arcEnd;
                s.endAngle = prevStep.endAngle;
                s.endVel = cals.maxVel;
                s.length = minR * arcAngle;
                s.dist = s.length;
                steps.add(s);

            } else {
                //last waypoint, end the path
                s = new Step();
                s.endPos = waypoint;
                s.endAngle = prevStep.endAngle;
                s.endVel = 0;
                s.length = Vector.subVectors(waypoint, prevStep.endPos).r;
                s.dist = s.length;
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

        double maxVel = cals.maxVel;

        ArrayList<Step> backStepList = new ArrayList<>();

        for(Tag tag : tags) {
            double targetIndex = tag.step - tagStep;
            while(targetIndex > 1){
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
            

            //angle tags modify the path
            if(tag.type == Type.ANGLE){
                double stepDist = currStep.length * targetIndex;
                Step s = new Step();
                s.endPos = Vector.subVectors(currStep.endPos, prevStep.endPos);
                s.endPos.r = stepDist;
                s.endPos.add(prevStep.endPos);
                s.length = stepDist;
                s.dist = s.length;
                currStep.length -= stepDist;
                if(tag.type == Type.ANGLE){
                    currStep.endAngle = tag.value;
                    s.endAngle = tag.value;
                    s.endVel = maxVel;
                    
                    //modify all steps between currStep and angleStep to perform the rotation
                    backStepList.add(s);
                    double totalLength = s.length;
                    Step step = stepIter.previous();
                    while(!step.equals(angleStep)){
                        if(!step.isArc || allowArcAngle){
                            backStepList.add(step);
                            totalLength += step.length;
                        }
                        step = stepIter.previous();
                    }
                    double anglePerLen = Angle.normRad(tag.value - angleStep.endAngle) / totalLength;
                    double startAngle = s.endAngle;
                    double angleDist = 0;
                    //note this loops backwards (first index is last step)
                    for(Step backStep : backStepList){
                        if(!backStep.isArc || allowArcAngle){
                            startAngle -= anglePerLen*angleDist;
                            backStep.endAngle = startAngle;
                            angleDist += backStep.length;
                            backStep.length += Math.abs(anglePerLen*backStep.length*botRadius);
                        }
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
                    if(stepIter.hasNext()) {
                        currStep = stepIter.next();
                        stepIdx++;
                        tagStep++;
                    }
                    
                } 

            } else {
                if(tag.type == Type.VEL){
                    Tag t = new Tag();
                    t.type = tag.type;
                    t.value = tag.value;
                    t.step = tag.step - tagStep + stepIdx;
                    vList.add(t);
                } else {
                    //add tag to tags list so it runs during execute
                    Tag t = new Tag();
                    t.type = tag.type;
                    t.value = tag.value;
                    //this math will need to change if we are ok with vision/flags on a curve
                    t.step = tag.step - tagStep + stepIdx;
                    tagList.add(t);
                }
            }
        }


        //loop through the velocity tags and calculate start/end velocities for each step
        stepIter = path.listIterator();
        ListIterator<Tag> vTagIter = vList.listIterator();
        Tag currTag = vTagIter.next();

        prevStep = stepIter.next();
        stepIdx = 2;
        while(stepIter.hasNext()){
            currStep = stepIter.next();
            
            //add new step for the velocity change
            if(vTagIter.hasNext() && currTag.step - stepIdx < 1){
                double stepDist = currStep.dist * (currTag.step - stepIdx);
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
                    maxVel = currTag.value;
                    maxVel = Math.min(maxVel, cals.maxVel);

                    s.endVel = Math.sqrt(prevStep.endVel*prevStep.endVel + 2*cals.maxAccel*s.length);
                    s.endVel = Math.min(maxVel, s.endVel);

                    currStep.endVel = Math.sqrt(s.endVel*s.endVel + 2*cals.maxAccel*currStep.length);
                    currStep.endVel = Math.min(maxVel, currStep.endVel);
                } else {
                    //decel, look back to make sure we can slow down
                    maxVel = currTag.value;
                    currStep.endVel = maxVel;
                    
                    s.endVel = maxVel;
                    Step step = stepIter.previous();
                    double lowestVel = Math.sqrt(step.endVel*step.endVel - 2*cals.maxAccel*s.length);
                    while(lowestVel > maxVel) {

                    }
                }
            }

            //assume accel until proven otherwise
            currStep.endVel = Math.sqrt(prevStep.endVel*prevStep.endVel + 2*cals.maxAccel*currStep.length);
            currStep.endVel = Math.min(maxVel, currStep.endVel);
        }

        //decel to zero for the last step


        return tagList;
    }
}
