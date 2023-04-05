package frc.robot.subsystems.Inputs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.PositionProvider;
import frc.robot.util.Vector;

public class Inputs extends SubsystemBase{
    
    RobotContainer r;
    InputCal cal;

    GenericEntry[] buttons = new GenericEntry[27];

    public Joystick controller;
    public Joystick cBoard;
    public Joystick cBoardTwo;

    public boolean disableAutoGather = true;
    boolean useButtonsTab = false;
    boolean[] prevButtonStates = new boolean[27];

    Joystick cBoardHead;//The only point of this object is to differentiate between the three cbs

    public enum joystickTypes{
        FLYSKY, GAMEPAD, NONE, CONTROL_BOARD_PT_ONE, CONTROL_BOARD_PT_TWO, CONTROL_BOARD_HEAD
    }
    public joystickTypes controllerType = joystickTypes.NONE;//differs between flysky vs. gamepad

    public Inputs(RobotContainer r, InputCal cal){
        this.r = r;
        this.cal = cal;

        for(int i = 0; i < 27; i++){
            prevButtonStates[i] = false;

            int iplusone = i + 1;
            buttons[i] = Shuffleboard.getTab("Buttons").add("" + iplusone, false).withPosition(i%9, i/9).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        }
    }

    GenericEntry levelNT = Shuffleboard.getTab("Comp").add("Sel Level", Level.NONE.toString()).withPosition(0, 2).getEntry();
    GenericEntry posNT = Shuffleboard.getTab("Comp").add("Sel Pos", Position.NONE.toString()).withPosition(2, 2).getEntry();
    GenericEntry zoneNT = Shuffleboard.getTab("Comp").add("Sel Zone", Zone.NONE.toString()).withPosition(1, 2).getEntry();

    joystickTypes[] portStatus = {joystickTypes.NONE, joystickTypes.NONE, joystickTypes.NONE, joystickTypes.NONE};
    joystickTypes[] prevPortStatus = {joystickTypes.NONE, joystickTypes.NONE, joystickTypes.NONE, joystickTypes.NONE};
    public void periodic(){
        levelNT.setString(selectedLevel.toString());
        posNT.setString(selectedPosition.toString());
        zoneNT.setString(selectedZone.toString());  
        
        //joystick auto-detection logic
        for(int i = 0; i < 3; i++){
            if(DriverStation.getJoystickName(i).contains("NV14")){
                controllerType = joystickTypes.FLYSKY;
                portStatus[i] = joystickTypes.FLYSKY;
            }else if(DriverStation.getJoystickName(i).contains("Gamepad or sumthin idk")){
                controllerType = joystickTypes.GAMEPAD;
                portStatus[i] = joystickTypes.GAMEPAD;
            } else if(DriverStation.getJoystickName(i).contains("I-PAC")){
                if(cBoard == null){
                    portStatus[i] = joystickTypes.CONTROL_BOARD_PT_ONE;
                } else if(cBoardTwo == null){
                    portStatus[i] = joystickTypes.CONTROL_BOARD_PT_TWO;
                }
            } else {
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
                    case CONTROL_BOARD_PT_ONE:
                        System.out.println("Control Board Pt One detected at port " + i);
                        cBoard = new Joystick(i);
                        break;
                    case CONTROL_BOARD_HEAD:
                        cBoardHead = new Joystick(i);
                    case CONTROL_BOARD_PT_TWO:
                        System.out.println("Control Board Pt Two detected at port " + i);
                        cBoardTwo = new Joystick(i);
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

    public Trigger cameraModeTrigger = new Trigger(this::cameraMode);

    public boolean cameraMode(){
        if(controller != null){
            return controller.getRawButton(cal.CAMERA_MODE[controllerType.ordinal()]);
        } else {
            return false;
        }
    }

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

    public Trigger fieldAlignRight = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            /*if(controller != null){
                return controller.getRawButton(cal.ALIGN_RIGHT[controllerType.ordinal()]);
            } else {
                return false;
            }*/
            return selectedPosition == Position.LEFT;
        }
    });

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

    public Trigger resetArm = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return controller.getRawButton(12);
            } else {
                return false;
            }
        }
    });

    public Trigger alignMode = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return controller.getRawButton(cal.ALIGN_MODE[controllerType.ordinal()]);
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

    public Trigger parkMode = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return controller.getRawButton(cal.PARK_MODE[controllerType.ordinal()]);
            } else {
                return false;
            }
        }
    });

    public boolean inchMode = false;
    public void setInchMode(boolean mode){
        inchMode = mode;
    }

    // ------------- End drive inputs ------------- //


    // ------------- Manipulator inputs ------------- //

    public boolean getAutoMode(){
        if(controller != null){
            return controller.getRawButton(cal.AUTO_MODE[controllerType.ordinal()]);
        } else {
            return false;
        }
    }

    public boolean getManualMode(){
        if(controller != null){
            return controller.getRawButton(cal.MANUAL_MODE[controllerType.ordinal()]);
        } else {
            return false;
        }
    }

    public Vector getGatherPosition(){
        if (isShelf()){
            return r.arm.cals.positionGatherShelf;
        }else{
            if(isCube()){
                if(DriverStation.isTeleop()){
                    return r.arm.cals.positionCubeGatherFloor;
                } else {
                    return r.arm.cals.positionCubeGatherFloorLongBoi;
                }
            } else {
                return r.arm.cals.positionConeGatherFloor;
            }
        }
    }

    public Trigger autoScore = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return controller.getRawAxis(cal.SCORE_TRIGGER[controllerType.ordinal()]) > 0.5;
            } else {
                return false;
            }
        }
    });

    public Trigger autoGather = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(controller != null){
                return getLeftTrigger() && !autoScore.getAsBoolean();
            } else {
                return false;
            }
        }
    });

    public boolean getLeftTrigger(){
        if(controller != null){
            return controller.getRawAxis(cal.GATHER_TRIGGER[controllerType.ordinal()]) > 0.5;
        } else {
            return false;
        }
    }

    public enum ManScoreMode {UP, SCORE};
    public ManScoreMode scoreMode = ManScoreMode.UP;

    public void setMode(ManScoreMode mode){
        scoreMode = mode;
    }
    public void toggleMode(){
        if(scoreMode == ManScoreMode.UP){
            scoreMode = ManScoreMode.SCORE;
        } else {
            scoreMode = ManScoreMode.UP;
        }
    }

    public PositionProvider armScorePos = new PositionProvider() {

        @Override
        public Vector getPosition() {
            Vector pos;
            if(scoreMode == ManScoreMode.UP){
                if(r.inputs.isCube()){
                    switch(selectedLevel){
                        case BOTTOM:
                            pos = r.arm.cals.positionCubeLow;
                            break;
                        case MIDDLE:
                            pos = r.arm.cals.positionCubeMed;
                            break;
                        case TOP:
                            pos = r.arm.cals.positionCubeHi;
                            break;
                        case NONE:
                            pos = r.arm.cals.positionCubeHi;
                            break;
                        default:
                            pos = r.arm.cals.positionCubeHi;
                            break;
                    }
                } else {
                    switch(selectedLevel){
                        case BOTTOM:
                            pos = r.arm.cals.positionConeLowRelease;
                            break;
                        case MIDDLE:
                            pos = r.arm.cals.positionConeMedHold;
                            break;
                        case TOP:
                            pos = r.arm.cals.positionConeHiHold;
                            break;
                        case NONE:
                            pos = r.arm.cals.positionConeHiHold;
                            break;
                        default:
                            pos = r.arm.cals.positionConeHiHold;
                            break;
                    }
                }
            } else {
                if(r.inputs.isCube()){
                    switch(selectedLevel){
                        case BOTTOM:
                            pos = r.arm.cals.positionCubeLow;
                            break;
                        case MIDDLE:
                            pos = r.arm.cals.positionCubeMed;
                            break;
                        case TOP:
                            pos = r.arm.cals.positionCubeHi;
                            break;
                        case NONE:
                            pos = r.arm.cals.positionCubeHi;
                            break;
                        default:
                            pos = r.arm.cals.positionCubeHi;
                            break;
                    }
                }else{
                    switch(selectedLevel){
                        case BOTTOM:
                            pos = r.arm.cals.positionConeLowRelease;
                            break;
                        case MIDDLE:
                            pos = r.arm.cals.positionConeMedRelease;
                            break;
                        case TOP:
                            pos = r.arm.cals.positionConeHiRelease;
                            break;
                        case NONE:
                            pos = r.arm.cals.positionConeHiRelease;
                            break;
                        default:
                            pos = r.arm.cals.positionConeHiRelease;
                            break;
                    }
                }
            }

            return pos;
        }
    };

    public boolean scoringSlowMode = false;
    public void slowModeTrue(){
        scoringSlowMode = true;
    }
    public void slowModeFalse(){
        scoringSlowMode = false;
    }
    

    // ------------- End manipulator inputs ------------- //


    // ------------- Control Board inputs ------------- //
    
    public boolean getFieldMode(){
        if(DriverStation.isFMSAttached()) return true;

        if(cBoardTwo != null){
            return cBoardTwo.getRawButton(cal.FIELD_MODE);
        } else {
            return true;
        }
    }

    public Trigger shift = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoardTwo != null){
                return cBoardTwo.getRawButton(cal.SHIFT);
            } else {
                return false;
            }
        }
    });

    public boolean isCube(){
        if(DriverStation.isAutonomous()) return true;
        if(cBoardTwo != null){
            return !cBoardTwo.getRawButton(cal.CUBE_V_CONE);
        } else {
            return true;
        }
    }

    public Trigger isShelfTrigger = new Trigger(this::isShelf);

    public boolean isShelf(){
        if(DriverStation.isAutonomous()) return false;
        if(cBoardTwo != null){
            return cBoardTwo.getRawButton(cal.SHELF_V_FLOOR);
        } else {
            return false;
        }
    }

    public Trigger intake = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoardTwo != null){
                return cBoardTwo.getRawButton(cal.INTAKE);
            } else {
                return false;
            }
        }
    });

    public Trigger gather = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoardTwo != null){
                return cBoardTwo.getRawButton(cal.GATHER);
            } else {
                return false;
            }
        }
    });

    public Trigger jogUp = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getPOV() == 0;
            } else {
                return false;
            }
        }
    });

    public Trigger jogDown = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getPOV() == 180;
            } else {
                return false;
            }
        }
    });

    public Trigger jogLeft = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getPOV() == 270;
            } else {
                return false;
            }
        }
    });

    public Trigger jogRight = new Trigger(new BooleanSupplier() {
        public boolean getAsBoolean(){
            if(cBoard != null){
                return cBoard.getPOV() == 90;
            } else {
                return false;
            }
        }
    });

    public boolean determineLeftAlignment(){
        if(selectedZone.ordinal() == 1){
            return true;
        } else if(selectedZone.ordinal() == 2){
            if(selectedPosition.ordinal() == 1){
                return false;
            } else {
                return true;
            }
        } else {
            return false;
        }
    }

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
    public enum Position {NONE, RIGHT, CENTER, LEFT};
    public enum GamePiece {EITHER, CONE, CUBE};

    public Level selectedLevel = Level.NONE;
    public Zone selectedZone = Zone.NONE;
    public Position selectedPosition = Position.NONE;
    public GamePiece selectedGamePiece = GamePiece.EITHER;

    public int buttonAssignment = -1;
    int prevButton = -1;

    public void scorePosition(){
        int idx = -1;
        //This is so messed up what they're making me do here :((
        if(cBoard != null && cBoardTwo != null){
            for(int i = 0; i < 10; i++){
                if(cBoard.getRawButton(cal.SCORE_POS_IDX[i])){
                    idx = i;
                }
            }
            for(int i = 10; i < 16; i++){
                if(cBoard.getRawAxis(cal.SCORE_POS_IDX[i]) > 0.1){
                    if(cal.SCORE_POS_IDX[i] == cal.SCORE_POS_IDX[i-1]){
                        idx = i;
                    }
                } else if(cBoard.getRawAxis(cal.SCORE_POS_IDX[i]) < -0.1){
                    if(cal.SCORE_POS_IDX[i] == cal.SCORE_POS_IDX[i+1]){
                        idx = i;
                    }
                }
            }
            if(cBoardTwo.getPOV() == 0){
                idx = 16;
            } else if(cBoardTwo.getPOV() == 180){
                idx = 17;
            } else if(cBoardTwo.getPOV() == 270){
                idx = 18;
            } else if(cBoardTwo.getPOV() == 90){
                idx = 19;
            }
            for(int i = 20; i < 24; i++){
                if(cBoardTwo.getRawAxis(cal.SCORE_POS_IDX[i]) > 0.1){
                    if(cal.SCORE_POS_IDX[i] == cal.SCORE_POS_IDX[i-1]){
                        idx = i;
                    }
                } else if(cBoardTwo.getRawAxis(cal.SCORE_POS_IDX[i]) < -0.1){
                    if(cal.SCORE_POS_IDX[i] == cal.SCORE_POS_IDX[i+1]){
                        idx = i;
                    }
                }
            }
            if(cBoardTwo.getRawAxis(cal.SCORE_POS_IDX[24]) < -0.1){
                idx = 24;
            }
            if(cBoardTwo.getRawButton(cal.SCORE_POS_IDX[25])){
                idx = 25;
            }
            if(cBoardTwo.getRawButton(cal.SCORE_POS_IDX[26])){
                idx = 26;
            }
        }
        if(useButtonsTab){
            for(int i = 0; i < 27; i++){
                if(buttons[i].getBoolean(false) == true && prevButtonStates[i] == false){
                    prevButtonStates[i] = true;
                    for(int toIdx = 0; toIdx < 27; toIdx++){
                        buttons[toIdx].setBoolean(i == toIdx);
                    }
                    idx = i;
                }
            }
        }

        if(idx != -1) buttonAssignment = idx + 1;
        SmartDashboard.putNumber("Button Assignment", buttonAssignment);
        if(buttonAssignment >= 1 && buttonAssignment <= 27){

            //level logic
            if(buttonAssignment <= 9){
                selectedLevel = Level.BOTTOM;
            } else if(buttonAssignment > 9 && buttonAssignment < 19){
                selectedLevel = Level.MIDDLE;
            } else {
                selectedLevel = Level.TOP;
            }

            //zone logic
            int horizontalIdx = buttonAssignment % 9;
            if(horizontalIdx <= 3 && horizontalIdx != 0){
                selectedZone = Zone.LEFT;
            } else if(horizontalIdx > 3 && horizontalIdx < 7) {
                selectedZone = Zone.COMMUNITY;
            } else {
                selectedZone = Zone.RIGHT;
            }
            
            int positionIdx = buttonAssignment % 3;
            if(positionIdx == 0) positionIdx = 3;
            SmartDashboard.putNumber("Position idx", positionIdx);

            //game piece logic
            if(positionIdx % 2 == 0){
                selectedGamePiece = GamePiece.CUBE;
            } else {
                selectedGamePiece = GamePiece.CONE;
            }

            //check for discrepancy between the switch and what we actually have
            //only force the correction when enabled or on the field
            boolean forceLegalPosition = DriverStation.isEnabled() || DriverStation.isFMSAttached();
            if(forceLegalPosition && isCube() != (selectedGamePiece == GamePiece.CUBE) && selectedGamePiece != GamePiece.EITHER && selectedLevel.ordinal() != 1){
                //Selects the nearest available score position on the same level in the same zone

                int fixedPositionIdx;
                //fix the coneVCube thing and change the index
                if(selectedGamePiece == GamePiece.CUBE){
                    selectedGamePiece = GamePiece.CONE;

                    fixedPositionIdx = positionIdx + 1;

                    buttonAssignment += 1;
                } else {
                    selectedGamePiece = GamePiece.CUBE;

                    fixedPositionIdx = 2;

                    if(positionIdx == 1){
                        buttonAssignment += 1;
                    } else {
                        buttonAssignment -= 1;
                    }
                }

                selectedPosition = Position.values()[fixedPositionIdx];
            } else {
                selectedPosition = Position.values()[positionIdx];
            }

            if(prevButton > 0){
                r.lights.ledOutputSet(prevButton-1, false);
            }
            prevButton = buttonAssignment;
            r.lights.ledOutputSet(buttonAssignment-1, true);
            
        }
    }

    // ------------- End control board inputs ------------- //
}
