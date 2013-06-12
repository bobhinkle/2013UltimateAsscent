package Sensors;

import edu.wpi.first.wpilibj.*;

public class Potentiometer extends Thread implements PIDSource
{
    private AnalogChannel pot;
    private double shift, ratio;
    /*
    private Timer timer;
    private double rate, lastAngle;
    private static final double timePerCheck;
    */

    public Potentiometer(int analogport, double ratio, double shift)
    {
        pot = new AnalogChannel(analogport);
        this.shift = shift;
        this.ratio = ratio;
        //timer = new Timer();
    }

    public double getAngle()
    {
	    //System.out.println(pot.getValue());
        return (pot.getValue() - shift) * ratio;
    }

    /*
    public double getAngularRate()
    {
        double rate = timer.get();
        timer.reset();
        return rate;
    }
    */

    public double getValue()
    {
        return pot.getValue();
    }

    /*
    public void run()
    {
        while(true)
        {
            if (timer.get() > timePerCheck)
            {
                rate = (getAngle() - lastAngle) / timer.get();
                timer.reset();
                lastAngle = this.getAngle();
            }
        }
    }
    */

    public void setZero(double shift) { this.shift = shift; }
    
    public double getSin() { return Math.sin(Math.toRadians(getAngle())); }

    public double pidGet() { return getAngle(); }
}
