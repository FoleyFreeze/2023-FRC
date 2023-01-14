package frc.robot.subsystems.Inputs;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class Inputs extends SubsystemBase{
    
    RobotContainer r;
    InputCal cal;

    Joystick controller;
    Joystick cBoard;

    public enum controllerTypes{
        FLYSKY, GAMEPAD, NONE
    }
    public controllerTypes type;

    public Inputs(RobotContainer r, InputCal cal){
        this.r = r;
        this.cal = cal;

        controller = new Joystick(0);
        cBoard = new Joystick(1);
    }

    public void periodic(){
        if(controller.getName().contains("NV14")){
            type = controllerTypes.FLYSKY;
        } else if(controller.getName().contains("gamepad")) {
            type = controllerTypes.GAMEPAD;
        } else {
            type = controllerTypes.NONE;
        }
    } 

    public boolean getFieldOrient(){
        return controller.getRawButton(cal.FIELD_ORIENT[type.ordinal()]);
    }

    public double getJoystickX(){
        return controller.getRawAxis(cal.L_JOYSTICK_X[type.ordinal()]);
    }

    public double getJoystickY(){
        return controller.getRawAxis(cal.L_JOYSTICK_Y[type.ordinal()]);
    }

    public Vector getJoystickXY(){
        return Vector.fromXY(getJoystickX(), getJoystickY());
    }

    public double getJoystickZR(){
        return controller.getRawAxis(cal.R_JOYSTICK_X[type.ordinal()]);
    }
}
