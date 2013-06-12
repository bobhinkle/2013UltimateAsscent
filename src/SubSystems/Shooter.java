/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package SubSystems;

import Sensors.MotorController;
import Sensors.SuperEncoder;
import Team1323Robot.Robot;
import Team1323Robot.Team1323RobotFSM;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author xpsl05x
 */
public class Shooter
{
    private MotorController motor;
    private SuperEncoder enc;
    private Solenoid trigger;
    private Solenoid magLoader;
    
    private static final double OK_ERROR = 50; //TODO Calibrate
    private double goal = 0;
    private static Shooter instance = null;
    private boolean magInStowPosition = true;
    private boolean isLoading = true;
    private Robot robot;
    private Team1323RobotFSM fsm;
    public Lights lights;
    public Shooter(){
        trigger = new Solenoid(Ports.TRIGGER);
        triggerIn();
        enc = new SuperEncoder(Ports.SHOOTERENC,Ports.SHOOTERENC+1,false,1);
        enc.start();
        if(Constants.USE_TALONS){
            motor = new MotorController(Constants.TALON_CONTROLLER,Ports.SHOOTER_1);
        }else{
            motor = new MotorController(Constants.VICTOR_CONTROLLER,Ports.SHOOTER_1);
        }
        magLoader = new Solenoid(Ports.MAG_LOADER);
        lights = Lights.getInstance();
        upMag();
    }
    public static Shooter getInstance()
    {
        if( instance == null )
            instance = new Shooter();
        return instance;
    }
    public void loadParts(){
        robot = Robot.getInstance();
        fsm = Team1323RobotFSM.getInstance();
    }
    /** Used to interface with a PID controller
     *
     * @param output value for the PID controller to set the motors to
     */
    public void set(double power){ motor.set(power);}
    
    
    public double getGoal(){return goal;}
    public double getShooterSpeed(){return enc.getRPM();}
    public void updateEnc(){enc.update();}
    public void shoot(){
        triggerIn();
        Timer.delay(0.5);
        triggerOut();
        Timer.delay(0.55);
    }
    public void fastShoot(){
        triggerIn();
        Timer.delay(0.3);
        triggerOut();
        Timer.delay(0.35);
    }
    public void slowShot(){
        triggerIn();
        Timer.delay(0.3);
        triggerOut();
        Timer.delay(0.4);
    }
    public boolean onGoal() { 
        if(goal > 6000){
            return true;
        }else{
            return Util.onTarget(goal, enc.rpm,OK_ERROR); 
        }
    }

    public void triggerOut(){
        trigger.set(!Constants.TRIGGER_FIRE);
    }
    public void triggerIn(){
        trigger.set(Constants.TRIGGER_FIRE);
    }
    public void stowFrisbees(){
        isLoading = false;
        upMag();
    }
    public boolean getMagInStowPosition(){
        return magInStowPosition;
    }
    public void upMag(){ 
        magLoader.set(Constants.MAG_STOWED);
        magInStowPosition = true;
    }
    public void downMag(boolean force){
        if(force){
            magLoader.set(!Constants.MAG_STOWED); 
            magInStowPosition = false;
        }else{
            if((fsm.getGoalState() == Team1323RobotFSM.FLOOR_INTAKE) || (fsm.getGoalState() == Team1323RobotFSM.MAG_TO_LOWER)){
                if((robot.lift.getHeight() == 0) && (robot.wrist.getAngle() <= Constants.WRIST_MIN_ANGLE + 2.0)){
                    magLoader.set(!Constants.MAG_STOWED); 
                    magInStowPosition = false;
                }else{
                    System.out.println(robot.lift.getHeight() + " " + robot.wrist.getAngle());
                }
            }
        }
    }
    public void switchMag(){
        if(magInStowPosition){
            downMag(true);
        }else{
            upMag();
        }
    }
    
}