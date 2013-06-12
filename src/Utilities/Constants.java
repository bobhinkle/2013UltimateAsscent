/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Utilities;

/**
 *
 * @author xpsl05x
 */
public class Constants {
    
    public static final int     MAXFRISBEES = 4; // Maximum balls allowed to be loaded
    public static final boolean LOW_GEAR  = true; // Drivetrain low gear
    public static final boolean HIGH_GEAR = false; // Drivetrain high gear
    public static final boolean MAG_STOWED = false;
    public static final double MIN_DT_POWER = 0.2;
    
    public static final double TURN_KP = 0.04; //0.020
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.03;
    public static final double TURN_KP_2 = 0.055;
    public static final double TURN_KI_2 = 0.0;
    public static final double TURN_KD_2 = 0.0;
    public static final double TURN_KP_3 = 0.11;
    public static final double TURN_KI_3 = 0.0;
    public static final double TURN_KD_3 = 0.0;
    public static final double TURN_KP_LOW_L = 0.07;
    public static final double TURN_KP_LOW_R = 0.07;
    public static final double TURN_ON_TARGET_DEG = 0.5;
    public static final double ADJUST_TURN_DEG = 6; //6
     
    public static final double DIST_KP = 0.185; // 0.2
    public static final double DIST_KI = 0.0004; // 0
    public static final double DIST_KD = 10.0; // 1.5
    public static final double STRAIGHT_KP = 0.012;
    public static final double STRAIGHT_KI = 0.0;
    public static final double STRAIGHT_KD = 0.1;
    public static final double DISTANCE_TOLERANCE = 1.0;
    
    public static final double ELEVATOR_MAX_HEIGHT  = 10;   // MAXIMUM ELEVATOR HEIGHT
    public static final double ELEVATOR_MIN_HEIGHT  = -10;
    public static final double ELEVATOR_STOW        = ELEVATOR_MIN_HEIGHT;
    public static final double ELEVATOR_UP_AND_OVER_HEIGHT = 11;
    public static final double ELEVATOR_P = 0.30;
    public static final double ELEVATOR_I = 0.0;
    public static final double ELEVATOR_D = 0.01;
    public static final double ELEVATOR_AUTON_FLOOR_SHOT = 10.0;
    public static final double ELEVATOR_FLOOR_SHOT = 10.0;
    public static final double ELEVATOR_HUMAN_LOAD = 0.0;
    public static final double ELEVATOR_MIN_SHOOTER_LIFT_HEIGHT = 3.0;
    public static final double ELEVATOR_MIN_POWER = 0.0; //0.0 
    public static final double ELEVATOR_MAX_POWER = 0.7; //0.75
    public static final double ELEVATOR_TOLERANCE = 0.0;
    
    public static final double SHOOTER_P = 0.02;
    public static final double SHOOTER_D = 0.0;  
    public static final double SHOOTER_MIN_RPM = 0;
    public static final double SHOOTER_MAX_RPM = 6200; 
    public static final double SHOT_DELAY = 0.5;
    public static final double SHOOTER_KP = 0.001;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.001;
    public static final double SHOOTER_KFFV = 1/SHOOTER_MAX_RPM; //0.005
    public static final double SHOOTER_ON_TARGET_SPEED = 200;
    
    
    public static final double WRIST_MIN_ANGLE = 101.022;
    public static final double WRIST_MAX_ANGLE  = 90.0 + WRIST_MIN_ANGLE;
    public static final double WRIST_LOADING_ANGLE = WRIST_MIN_ANGLE;
    public static final double WRIST_P = 0.1; //0.15
    public static final double WRIST_I = 0.0;
    public static final double WRIST_D = 0.002;
    public static final double WRIST_TOLERANCE = 0.7;
    public static final double WRIST_MAX_POWER = 0.9;
    public static final double WRIST_MIN_POWER = 0.0;
    public static final double WRIST_PLUNGER_ANGLE = 5.0 + WRIST_MIN_ANGLE;
    
    public static final double WRIST_MAX_ANGLE_FOR_STOWED_ELEVATOR = WRIST_MIN_ANGLE + 5;    
    public static final double WRIST_FLOOR_SHOT_ANGLE  = 24.0 + WRIST_MIN_ANGLE; //24
    public static final double WRIST_SIDE_SHOT_ANGLE = 24.5 + WRIST_MIN_ANGLE; //24.5
    public static final double WRIST_AUTON_UNDER_PYRAMID_ANGLE = WRIST_MIN_ANGLE + 19.0; 
    public static final double WRIST_UNDER_PYRAMID_SHOT_ANGLE = WRIST_MIN_ANGLE + 26.75; // add 2 degrees if elevator is down original is +2.0
    public static final double WRIST_HUMAN_LOAD_ANGLE = WRIST_MIN_ANGLE + 5.0;
    public static final double WRIST_FULL_FIELD_ANGLE = 6 + WRIST_MIN_ANGLE;
    public static final double WRIST_AUTON_BACK_NO_LIFT_ANGLE = WRIST_MIN_ANGLE + 20.5; //25
    public static final double WRIST_AUTON_FRONT_OF_PYRAMID = WRIST_MIN_ANGLE + 36.5;
    public static final double WRIST_CLIMBER_ANGLE = WRIST_MIN_ANGLE + 85.0;
    public static final double WRIST_DUMP_ANGLE = WRIST_MIN_ANGLE + 70.0;
    public static final double INTAKE_IR_CLEAR  = 1.11;
    public static final double INTAKE_IR_FILLED = 1.6;
    public static final boolean TRIGGER_FIRE = false;
    
//    public static final double SHOOTER_SHOT = 3100; 
    
    public static final double DRIVE_DISTANCE_PER_PULSE = 0.03420833;
    public static final double ELEVATOR_DISTANCE_PER_PULSE = 0.007263889;
    public static final double TOPKEY_TO_BRIDGE = 150;
    public static final double TOPKEY_TO_FENDER = 104;
    
    public static final double VOLTS_TO_PSI = 53.18;
    
    public static final int TALON_CONTROLLER = 1;
    public static final int VICTOR_CONTROLLER = 2;
    
    public static final boolean USE_TALONS = true;
    
    public static final boolean TUNING_MODE = false;
    
    public static final boolean BYPASS_FRISBEE_DETECTOR = true;
    
    public static final int BOOTUP = 1;
    public static final int READY  = 2;
    public static final int INTAKEON = 3;
    public static final int NOTREADYTOSHOOT = 4;
    public static final int READYTOSHOOT    = 5;
    
}
