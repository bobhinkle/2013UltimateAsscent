/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Sensors;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;

/**
 *
 * @author xpsl05x
 */
public class MotorController {
    
    private Talon tMotor;
    private Victor vMotor;
    private int type = 1;
    public MotorController(int controllerSelect,int port){
        switch(controllerSelect){
            case 1:
                tMotor = new Talon(port);
                break;
            case 2:
                vMotor = new Victor(port);
                break;
        }
        type = controllerSelect;
    }
    public void set(double speed){
        switch(type){
            case 1:
                tMotor.set(speed);
                break;
            case 2:
                vMotor.set(speed);
                break;
        }
    }
}
