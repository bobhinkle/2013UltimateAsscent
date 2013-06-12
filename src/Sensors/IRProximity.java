/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Sensors;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.PIDSource;

/**
 *
 * @author xpsl05x
 */
public class IRProximity extends Thread implements PIDSource{
    private AnalogChannel sensor;
    private double shift, ratio;
    
    public IRProximity(int port){
        sensor = new AnalogChannel(port);
    }
    public double pidGet() {
        return sensor.getVoltage();
    }
    public double getDistance(){
        return sensor.getVoltage();
    }
}
