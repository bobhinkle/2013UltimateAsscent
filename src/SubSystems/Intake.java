/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package SubSystems;

import Sensors.MotorController;
import Utilities.Constants;
import Utilities.Ports;

/**
 *
 * @author xpsl05x
 */
public class Intake {
    private MotorController frontRoller,backRoller;
    private double frontMaxSpeed = -1;
    private double backMaxSpeed = -1;
    private static Intake instance = null;
    public Intake(){
        if(Constants.USE_TALONS){
            frontRoller = new MotorController(Constants.TALON_CONTROLLER,Ports.INTAKE_ROLLER_MOTOR_TOP);
            backRoller  = new MotorController(Constants.TALON_CONTROLLER,Ports.INTAKE_ROLLER_MOTOR);
        }else{
            frontRoller = new MotorController(Constants.VICTOR_CONTROLLER,Ports.INTAKE_ROLLER_MOTOR_TOP);
            backRoller  = new MotorController(Constants.VICTOR_CONTROLLER,Ports.INTAKE_ROLLER_MOTOR);
        }
    }
    
    public static Intake getInstance()
    {
        if( instance == null )
            instance = new Intake();
        return instance;
    }
    
    public void frontForward(){
        frontRoller.set(frontMaxSpeed);
    }
    public void frontReverse(){
        frontRoller.set(-frontMaxSpeed);
    }
    public void frontStop(){
        frontRoller.set(0);
    }
    public void backForward(){
        backRoller.set(backMaxSpeed);
    }
    public void backReverse(){
        backRoller.set(-backMaxSpeed);
    }
    public void backStop(){
        backRoller.set(0);
    }
    
    public void allForward(){
        frontForward();
        backForward();
    }
    public void allReverse(){
        frontReverse();
        backReverse();
    }
    public void allStop(){
        frontStop();
        backStop();
    }
}
