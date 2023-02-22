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

    public enum joystickTypes{
        FLYSKY, GAMEPAD, NONE, CONTROL_BOARD
    }
    public joystickTypes controllerType = joystickTypes.NONE;//differs between flysky vs. gamepad

    public Inputs(RobotContainer r, InputCal cal){
        this.r = r;
        this.cal = cal;
    }

    joystickTypes[] portStatus = {joystickTypes.NONE, joystickTypes.NONE, joystickTypes.NONE, joystickTypes.NONE};
    joystickTypes[] prevPortStatus = {joystickTypes.NONE, joystickTypes.NONE, joystickTypes.NONE, joystickTypes.NONE};
    public void periodic(){
        
        //joystick auto-detection logic
        for(int i = 0; i < 3; i++){
            if(DriverStation.getJoystickName(i).contains("NV14")){
                controllerType = joystickTypes.FLYSKY;
                portStatus[i] = joystickTypes.FLYSKY;
            }else if(DriverStation.getJoystickName(i).contains("Gamepad or sumthin idk")){
                controllerType = joystickTypes.GAMEPAD;
                portStatus[i] = joystickTypes.GAMEPAD;
            } else if(DriverStation.getJoystickName(i).contains("I-PAC")){
                portStatus[i] = joystickTypes.CONTROL_BOARD;
            } else{
                portStatus[i] = joystickTypes.NONE;
            }
            if(portStatus[i] != prevPortStatus[i]){
                switch(portStatus[i]){
                    case FLYSKY:
                        System.out.println("Flysky detected at port " + i);
                        controller = new Joystick(i);
                        break;
                    case GAMEPAD:
                        System.out.println("Gamepad detected at port " + i);
                        controller = new Joystick(i);
                        break;
                    case CONTROL_BOARD:
                        System.out.println("Control Board detected at port " + i);
                        cBoard = new Joystick(i);
                        break;
                    case NONE:
                        if(controller.getPort() == i){
                            controller = null;
                        } else if(cBoard.getPort() == i){
                            cBoard = null;
                        }
                        break;
                }
            }
            prevPortStatus[i] = portStatus[i];
        }
        

        scorePosition();
    } 


    // ------------- Drive inputs ------------- //

    public boolean getFieldOrient(){
        if(controller != null){
            return controller.getRawButton(cal.FIELD_ORIENT[controllerType.ordinal()]);
        } else {
            return false;
        }
    }

    public boolean getFieldAlign(){
        if(controller != null){
            //TODO: make this
            return false;//controller.getRawButton(cal.FIELD_ORIENT[controllerType.ordinal()]);
        } else {
            return false;
        }
    }

    public double getJoystickX(){
        if(controller != null){
            double value = controller.getRawAxis(cal.L_JOYSTICK_X[controllerType.ordinal()]);
            if(Math.abs(value) < 0.08) value = 0;
            return value;
        } else {
            return 0;
        }
    }

    public double getJoystickY(){
        if(controller != null){
            double value = controller.getRawAxis(cal.L_JOYSTICK_Y[controllerType.ordinal()]);
            if(Math.abs(value) < 0.08) value = 0;
            return value;
        } else {
            return 0;
        }
    }

    public Vector getJoystickXY(){
        return Vector.fromXY(getJoystickX(), getJoystickY());
    }

    public double getJoystickZR(){
        if(controller != null){
            double value = -controller.getRawAxis(cal.R_JOYSTICK_X[controllerType.ordinal()]);
            //Added deadband 
            if(Math.abs(value) < 0.12) value = 0;
            return value;
        } else {
            return 0;
        }
    }

    public Trigger resetSwerveZeros = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return controller.getRawButton(cal.RESET_WHEELS[controllerType.ordinal()]);
            } else {
                return false;
            }
        }
    });

    public Trigger resetAngle = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return controller.getRawButton(cal.RESET_ANG[controllerType.ordinal()]);
            } else {
                return false;
            }
        }
    });

    public Trigger resetPosition = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return controller.getRawButton(cal.RESET_POS[controllerType.ordinal()]);
            } else {
                return false;
            }
        }
    });

    public Trigger balanceMode = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return controller.getRawButton(cal.BALANCE_MODE[controllerType.ordinal()]);
            } else {
                return false;
            }
        }
    });

    // ------------- End drive inputs ------------- //


    // ------------- Manipulator inputs ------------- //

    public Vector getGatherPosition(){
        if (isShelf()){
            return r.arm.cals.positionGatherShelf;
        }else{
            return r.arm.cals.positionGatherFloor;
        }
    }

    public Trigger autoGather = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return controller.getRawAxis(cal.AUTO_PICKUP[controllerType.ordinal()]) > 0.5;
            } else {
                return false;
            }
        }
    });
    

    // ------------- End manipulator inputs ------------- //


    // ------------- Control Board inputs ------------- //
    
    public boolean getFieldMode(){
        if(cBoard != null){
            return cBoard.getRawButton(cal.FIELD_MODE);
        } else {
            return false;
        }
    }

    public boolean shift(){
        if(cBoard != null){
            return cBoard.getRawButton(cal.SHIFT);
        } else {
            return false;
        }
    }

    public boolean isCube(){
        if(cBoard != null){
            return cBoard.getRawButton(cal.CUBE_V_CONE);
        } else {
            return false;
        }
    }

    public boolean isShelf(){
        if(cBoard != null){
            return cBoard.getRawButton(cal.SHELF_V_FLOOR);
        } else {
            return false;
        }
    }

    public Trigger intake = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getRawButton(cal.INTAKE);
            } else {
                return false;
            }
        }
    });

    public Trigger gather = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getRawButton(cal.GATHER);
            } else {
                return false;
            }
        }
    });

    public Trigger jogUp = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getRawButton(cal.JOG_UP);
            } else {
                return false;
            }
        }
    });

    public Trigger jogDown = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getRawButton(cal.JOG_DOWN);
            } else {
                return false;
            }
        }
    });

    public Trigger jogLeft = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getRawButton(cal.JOG_LEFT);
            } else {
                return false;
            }
        }
    });

    public Trigger jogRight = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getRawButton(cal.JOG_RIGHT);
            } else {
                return false;
            }
        }
    });

    /* This is the temporary indexing to the physical positions
     * 
     *                 (Driver Station)
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
     */

    public enum Level {NONE, BOTTOM, MIDDLE, TOP};
    public enum Zone {NONE, LEFT, COMMUNITY, RIGHT};
    public enum Position {NONE, LEFT, CENTER, RIGHT};
    public enum GamePiece {EITHER, CONE, CUBE};

    public Level selectedLevel = Level.NONE;
    public Zone selectedZone = Zone.NONE;
    public Position selectedPosition = Position.NONE;
    public GamePiece selectedGamePiece = GamePiece.EITHER;

    //
    public void scorePosition(){
        int idx = -1;
        for(int i = 0; i < cal.SCORE_POS_IDX.length; i++){
            if(cBoard != null && cBoard.getRawButton(cal.SCORE_POS_IDX[i]) == true){
                idx = i;
            }
        }

        int buttonAssignment = idx + 1;
        if(buttonAssignment >= 1 && buttonAssignment <= 27){

            //level logic
            if(buttonAssignment <= 9){
                selectedLevel = Level.TOP;
            } else if(buttonAssignment > 9 && buttonAssignment < 19){
                selectedLevel = Level.MIDDLE;
            } else {
                selectedLevel = Level.BOTTOM;
            }

            //zone logic
            int horizontalIdx = buttonAssignment % 9;
            if(horizontalIdx <= 3){
                selectedZone = Zone.LEFT;
            } else if(horizontalIdx > 3 && horizontalIdx < 7) {
                selectedZone = Zone.COMMUNITY;
            } else {
                selectedZone = Zone.RIGHT;
            }
            
            int positionIdx = buttonAssignment % 3;

            //game piece logic
            if(positionIdx % 2 == 0){
                selectedGamePiece = GamePiece.CUBE;
            } else {
                selectedGamePiece = GamePiece.CONE;
            }

            //check for discrepancy between the switch and what we actually have
            if(isCube() != (selectedGamePiece == GamePiece.CUBE) && selectedGamePiece != GamePiece.EITHER){
                //Selects the nearest available score position on the same level in the same zone

                int fixedPositionIdx;
                //fix the coneVCube thing and change the index
                if(selectedGamePiece == GamePiece.CUBE){
                    selectedGamePiece = GamePiece.CONE;

                    fixedPositionIdx = positionIdx + 1;
                } else {
                    selectedGamePiece = GamePiece.CUBE;

                    fixedPositionIdx = 2;
                }

                selectedPosition = Position.values()[fixedPositionIdx];
            } else {
                selectedPosition = Position.values()[positionIdx];
            }
            
        } else {
            selectedLevel = Level.NONE;
            selectedZone = Zone.NONE;
            selectedPosition = Position.NONE;
        }
    }

    // ------------- End control board inputs ------------- //
}
