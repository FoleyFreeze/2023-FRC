package frc.robot.subsystems.Inputs;

public class InputCal {
    //controller indexes

    //flysky, gamepad, nothin'

    public final int[] L_JOYSTICK_X = {0, 0, 0};
    public final int[] L_JOYSTICK_Y = {1, 0, 0};
    public final int[] R_JOYSTICK_X = {4, 0, 0};
    public final int[] R_JOYSTICK_Y = {5, 0, 0};

    public final int[] GATHER_TRIGGER = {2, 0, 0};
    public final int[] SCORE_TRIGGER = {3, 0, 0};

    public final int[] AUTO_MODE = {2, 0, 0};
    public final int[] MANUAL_MODE = {3, 0, 0};

    public final int[] FIELD_ORIENT = {5, 0 ,0};
    public final int[] BALANCE_MODE = {4, 0, 0};

    public final int[] RESET_WHEELS = {11, 0, 0};
    public final int[] RESET_ANG = {10, 0, 0};
    public final int[] RESET_POS = {14, 0, 0};

    //control board indexes

    public final int FIELD_MODE = 6;
    public final int SHIFT = 3;
    public final int CUBE_V_CONE = 4;
    public final int SHELF_V_FLOOR = 5;
    public final int INTAKE = 1;
    public final int GATHER = 2;

    public final int[] SCORE_POS_IDX = {1, //1 BUTTON
                                        2, //2 BUTTON
                                        3, //3 BUTTON
                                        4, //4 BUTTON
                                        5, //5 BUTTON
                                        6, //6 BUTTON
                                        7, //7 BUTTON
                                        8, //8 BUTTON
                                        9, //9 BUTTON
                                        10, //10 BUTTON
                                        0, //11 AXIS
                                        0, //12 AXIS
                                        1, //13 AXIS
                                        1, //14 AXIS
                                        2, //15 AXIS
                                        2, //16 AXIS
                                        -1, //17 POV
                                        -1, //18 POV
                                        -1, //19 POV
                                        -1, //20 POV
                                        0, //21 AXIS
                                        0, //22 AXIS
                                        1, //23 AXIS
                                        1, //24 AXIS
                                        2, //25 AXIS
                                        9, //26 BUTTON
                                        10, //27 BUTTON
                                       };

    //lights stuff
    public final int LED_PORT = 0;
    public final int BUFFER_LENGTH = 200;
}
