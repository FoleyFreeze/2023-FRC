package frc.robot.subsystems.Inputs;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Inputs extends SubsystemBase{
    
    RobotContainer r;
    InputCal cal;

    Joystick controller;

    public enum controllerTypes{
        FLYSKY, GAMEPAD, NONE
    }
    public controllerTypes type;

    public Inputs(RobotContainer r, InputCal cal){
        this.r = r;
        this.cal = cal;

        controller = new Joystick(0);
    }

    public void periodic(){
        if(controller != null){
            if(controller.getName().contains("flysky")){
                type = controllerTypes.FLYSKY;
            } else {
                type = controllerTypes.GAMEPAD;
            }
        } else {
            type = controllerTypes.NONE;
        }
    } 

    public boolean getFieldOrient(){
        return controller.getRawButton(cal.FIELD_ORIENT[type.ordinal()]);
    }

    public double getJoystickX(){
        return controller.getRawAxis(cal.L_JOYSTICK[type.ordinal()]);
    }

    public double getJoystickY(){
        return controller.getRawAxis(cal.L_JOYSTICK[type.ordinal()]);
    }

    public double getJoystickZR(){
        return 0;
    }
}
