/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Sensors;
import edu.wpi.first.wpilibj.*;
/**
 *
 * @author xpsl05x
 */
public class UltraSonic extends Thread implements PIDSource{
    
    private AnalogChannel us;
    private double shift, ratio;
    
    
    public UltraSonic(int analogport){
        us = new AnalogChannel(analogport);
    }
    public double getDistance(){
        double distance = (us.getVoltage()/.0098);
        return distance;
    }
    public double getVoltage(){
        return us.getVoltage();
        
    }
    public double getValue(){
        return us.getValue();
    }
    public double pidGet() { return getDistance(); }
}
