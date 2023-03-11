package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class LearnArmOffset extends CommandBase{
    
    RobotContainer r;

    public LearnArmOffset(RobotContainer r){
        this.r = r;
        addRequirements(r.arm);
    }

    double startTime;
    double currentAngle;

    @Override
    public void initialize(){
        startTime = Timer.getFPGATimestamp();
        //get the arm position from a pot
        currentAngle = Math.toDegrees((r.arm.armPot.getVoltage() - r.arm.cals.armPotOffset) * r.arm.cals.armPotSlope);
        r.arm.angleMotor.setEncoderPosition(currentAngle);
        //angleMotor.setEncoderPosition(-11);

        System.out.println("Reset arm angle/extension");
        r.arm.jogOffset = new Vector(0,0);
    }

    @Override
    public void execute(){
        //the stendo should reset to a mid-ish position, 
        //but then on retraction it should hit the stop and relearn
        //stendoMotor.setEncoderPosition(cals.initialStendoPosition + getStendoPulleyOffset(currentAngle));
        SmartDashboard.putNumber("Stendo Current", r.arm.stendoMotor.getCurrent());
        r.arm.stendoMotor.setPower(-0.3);
        r.arm.moveArmOnly(Vector.fromDeg(0, currentAngle));
    }

    @Override
    public void end(boolean interrupted){
        r.arm.stendoMotor.setPower(0);
        if(!interrupted){
            r.arm.stendoMotor.setEncoderPosition(r.arm.cals.lengthMin + r.arm.getStendoPulleyOffset(currentAngle));
        }
    }

    @Override
    public boolean isFinished(){
        if(startTime + 0.4 < Timer.getFPGATimestamp()){
            return r.arm.stendoMotor.getCurrent() > 20;
        } else {
            return false;
        }
    }
}
