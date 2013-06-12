/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Sensors;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author xpsl05x
 */
public class AnalogPressure {
    private AnalogChannel sensor;
    private double voltsToPSI;
    private double volts = 0;
    public AnalogPressure(int port,double vtp){
        sensor = new AnalogChannel(port);
        voltsToPSI = vtp;
    }
    public double getPressure(){
        volts = sensor.getVoltage();
        SmartDashboard.putNumber("AIR", volts);
        return (volts-0.496)*voltsToPSI;
    }
}
