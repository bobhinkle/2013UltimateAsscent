package SubSystems;

import Utilities.Ports;
import edu.wpi.first.wpilibj.*;

/** Uses a separate thread to regulate the air pressure by running a spike when
 *  the pressure gauge isn't tripped
 *
 * @author Robotics
 */
public class Compressor
{
    private Relay           compressor;
    private DigitalInput    pressure;
    public boolean running = true;
    public Compressor()
    {
        compressor  = new Relay(Ports.COMPRESSOR_SPIKE, Relay.Direction.kForward);
        pressure    = new DigitalInput(Ports.COMPRESSOR_SWITCH);
    }

    /** Starts the thread that runs the compressor
     */
    public void run()
    {
        if (pressure.get())
                compressor.set(Relay.Value.kOff);
          else
                compressor.set(Relay.Value.kOn);
        
    }
}
