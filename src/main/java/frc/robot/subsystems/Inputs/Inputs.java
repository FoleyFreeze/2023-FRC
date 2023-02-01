package frc.robot.subsystems.Inputs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.util.Vector;

public class Inputs extends SubsystemBase{
    
    RobotContainer r;
    InputCal cal;

    public Joystick controller;
    public Joystick cBoard;

    public boolean joyStickConnected = false;
    public boolean cBoardConnected = false;

    public enum controllerTypes{
        FLYSKY, GAMEPAD, NONE
    }
    public controllerTypes type = controllerTypes.NONE;

    public Inputs(RobotContainer r, InputCal cal){
        this.r = r;
        this.cal = cal;
    }

    int prevIdx = -1;
    int controllerIdx = -1;
    public void periodic(){

        //joystick auto-detection logic
        int i = 0;
        for( ; i < 3; i++){
            if(DriverStation.isJoystickConnected(i)){
                controllerIdx = i;
                if(controller.getName().contains("NV14")){
                    controller = new Joystick(i);

                    type = controllerTypes.FLYSKY;
                    System.out.println("Flysky detected at port " + i);
                } else if(controller.getName().contains("Control board or sumthin idk") && cBoardConnected == false){
                    cBoard = new Joystick(i);

                    System.out.println("Control Board detected at port: " + i);
                } else {
                    controller = new Joystick(i);

                    type = controllerTypes.GAMEPAD;
                    System.out.println("Gamepad detected at port " + i);
                }
            }
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



    // ------------- End manipulator inputs ------------- //


    // ------------- Control Board inputs ------------- //
    
    public boolean getFieldMode(){//TODO: Make a standard global way of controlling drive power
        return cBoard.getRawButton(cal.FIELD_MODE);
    }

    public boolean shift(){
        return cBoard.getRawButton(cal.SHIFT);
    }

    public boolean isCube(){
        return cBoard.getRawButton(cal.CUBE_V_CONE);
    }

    public boolean isShelf(){
        return cBoard.getRawButton(cal.SHELF_V_FLOOR);
    }

    public Trigger intake = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            return cBoard.getRawButton(cal.INTAKE);
        }
    });

    public Trigger gather = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            return cBoard.getRawButton(cal.GATHER);
        }
    });

    public Trigger jogUp = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            return cBoard.getRawButton(cal.JOG_UP);
        }
    });

    public Trigger jogDown = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            return cBoard.getRawButton(cal.JOG_DOWN);
        }
    });

    public Trigger jogLeft = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            return cBoard.getRawButton(cal.JOG_LEFT);
        }
    });

    public Trigger jogRight = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            return cBoard.getRawButton(cal.JOG_RIGHT);
        }
    });

    /*                 (Driver Station)
     * ---- ---- ----   ---- ---- ----   ---- ---- ----
     * |01| |02| |03|   |04| |05| |06|   |07| |08| |09|
     * ---- ---- ----   ---- ---- ----   ---- ---- ----
     * ---- ---- ----   ---- ---- ----   ---- ---- ----
     * |10| |11| |12|   |13| |14| |15|   |16| |17| |18|
     * ---- ---- ----   ---- ---- ----   ---- ---- ----
     * ---- ---- ----   ---- ---- ----   ---- ---- ----
     * |19| |20| |21|   |22| |23| |24|   |25| |26| |27|
     * ---- ---- ----   ---- ---- ----   ---- ---- ----
     * 
     * This is the temporary indexing to the physical positions
     * 
     */

    public int selectedScoreLevel = 0;
    public int selectedZone = 0;
    public int selected;

    public void scorePosition(){
        int idx = -1;
        for(int i = 0; i < cal.SCORE_POS_IDX.length; i++){
            if(cBoard.getRawButton(cal.SCORE_POS_IDX[i]) == true){
                idx = i;
            }
        }
        int buttonAssignment = idx + 1;

    }

    // ------------- End control board inputs ------------- //
}
