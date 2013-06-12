/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Utilities;

/**
 *
 * @author xpsl05x
 */
public class Ports {
    // Digital Sidecar 1
    public static final int RIGHTENC    = 3; //USES 2 Ports ( 3-4)
    public static final int LEFTENC     = 1; //USES 2 Ports ( 1-2)
    public static final int SHOOTERENC  = 5; //USED FOR SHOOTER SPEED (5-6)
    public static final int LIFTENC     = 7; //USED FOR LIFT HEIGHT (7-8)
    public static final int ELELIMIT    = 9;
    public static final int STATE1      = 10;
    public static final int STATE2      = 11;
    public static final int STATE3      = 12;
    public static final int STATECHANGE = 13;
    public static final int STATECHANGEDETECT  = 14;
    
    public static final int COMPRESSOR_SWITCH = 14;
    
    //Digital Sidecar 1 I2C
    public static final int ACCEL = 2;
    
    //Digital Sidecar 1 PWM
    
    public static final int RIGHTDT              = 1; 
    public static final int LEFTDT               = 2; 
    public static final int SHOOTER_1            = 3; 
    public static final int INTAKE_ROLLER_MOTOR  = 4; 
    public static final int SHOOTER_WRIST_MOTOR  = 5;
    public static final int INTAKE_ROLLER_MOTOR_TOP = 6; 
    public static final int ELEVATOR_LIFT        = 7; 
    public static final int CAMERA_LIGHT         = 8;
    public static final int HUMANLOAD   = 8;
    
    
    //Digital Sidecar relays
    public static final int TURRET_LIGHT = 6;
    
    
    //Modules
    public static final int ANALOG   = 0;
    public static final int DIGITAL  = 1;
    public static final int SILENOID = 2;
    
    // Solenoids
    
    public static final int SHIFTER     = 1;
    public static final int MAG_LOADER  = 2;
    public static final int TRIGGER     = 3;
    public static final int CLIMBER     = 4;
    public static final int PUSHROD     = 6;
    // Analog
    public static final int GYRO            = 1;
    public static final int SHOOTER_ARM     = 2;
    public static final int ANALOG_PRESSURE = 3;
    public static final int AUTON_SELECT    = 4;
    
    //cRio Port
    public static final int CRIOPORT     = 1;
    
    //Relay
    public static final int COMPRESSOR_SPIKE = 1;
}
