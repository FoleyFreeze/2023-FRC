package frc.robot.subsystems.Inputs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class Inputs extends SubsystemBase{
    
    RobotContainer r;
    InputCal cal;

    public Joystick controller;
    Joystick cBoard;

    public enum controllerTypes{
        FLYSKY, GAMEPAD, NONE
    }
    public controllerTypes type = controllerTypes.NONE;

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


    // ------------- Drive inputs ------------- //

    public boolean getFieldOrient(){
        return controller.getRawButton(cal.FIELD_ORIENT[type.ordinal()]);
    }

    public double getJoystickX(){
        double value = controller.getRawAxis(cal.L_JOYSTICK_X[type.ordinal()]);
        if(Math.abs(value) < 0.08) value = 0;
        return value;
    }

    public double getJoystickY(){
        double value = controller.getRawAxis(cal.L_JOYSTICK_Y[type.ordinal()]);
        if(Math.abs(value) < 0.08) value = 0;
        return value;
    }

    public Vector getJoystickXY(){
        return Vector.fromXY(getJoystickX(), getJoystickY());
    }

    public double getJoystickZR(){
        double value = -controller.getRawAxis(cal.R_JOYSTICK_X[type.ordinal()]);
        //Added deadband 
        if(Math.abs(value) < 0.08) value = 0;
        return value;
    }

    public Trigger resetSwerveZeros = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            return controller.getRawButton(cal.RESET_WHEELS[type.ordinal()]);
        }
    });

    public Trigger resetAngle = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            return controller.getRawButton(cal.RESET_ANG[type.ordinal()]);
        }
    });

    public Trigger resetPosition = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            return controller.getRawButton(cal.RESET_POS[type.ordinal()]);
        }
    });

    // ------------- End drive inputs ------------- //


    // ------------- Manipulator inputs ------------- //
}
