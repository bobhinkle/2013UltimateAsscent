/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package SubSystems;

import Utilities.Constants;
import Utilities.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 *
 * @author xpsl05x
 */
public class Lights {
    private DigitalOutput state1;
    private DigitalOutput state2;
    private DigitalOutput state3;
    
    private DigitalOutput stateChange;
    private DigitalInput  stateChangeDetected;
    
    private static Lights instance = null;
    private boolean needsChecking = false;
    private int lastState = 0;
    public Lights(){
        state1 = new DigitalOutput(Ports.STATE1);
        state2 = new DigitalOutput(Ports.STATE2);
        state3 = new DigitalOutput(Ports.STATE3);
        stateChange = new DigitalOutput(Ports.STATECHANGE);
//        stateChangeDetected = new DigitalInput(Ports.STATECHANGEDETECT);
    }
    public static Lights getInstance()
    {
        if( instance == null )
            instance = new Lights();
        return instance;
    }
    public void checkIfDetected(){
        if(needsChecking){
//            if(stateChangeDetected.get()){
//                stateChange.set(false);
//                needsChecking = false;
//            }
        }
    }
    public void updateLights(int state){
        if(lastState != state){
            switch(state){
                case 1:
                    state1.set(true);
                    state2.set(false);
                    state3.set(false);
                    break;
                case 2:
                    state1.set(false);
                    state2.set(true);
                    state3.set(false);
                    break;
                case 3:
                    state1.set(true);
                    state2.set(true);
                    state3.set(false);
                    break;
                case 4:
                    state1.set(false);
                    state2.set(false);
                    state3.set(true);
                    break;
                case 5:
                    state1.set(true);
                    state2.set(false);
                    state3.set(true);
                    break;
                case 6:
                    state1.set(false);
                    state2.set(true);
                    state3.set(true);
                    break;
                case 7:
                    state1.set(true);
                    state2.set(true);
                    state3.set(true);
                    break;
            }
            stateChange.set(true);
            needsChecking = true;
        }
    }
}
