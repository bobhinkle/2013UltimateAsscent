/*
 
 * 
 * 
 */
package Team1323Robot;

import IO.TeleController;
import Sensors.AnalogPressure;
import Sensors.MotorController;
import Sensors.Vision;
import SubSystems.DistanceController;
import SubSystems.DriveTrain;
import SubSystems.Intake;
import SubSystems.Lights;
import SubSystems.Navigation;
import SubSystems.Shooter;
import Utilities.Constants;
import Utilities.DashboardData;
import Utilities.Output;
import Utilities.Ports;
import SubSystems.ElevatorLoop;
import SubSystems.ShooterSpeedController;
import SubSystems.TurnController;
import SubSystems.Wrist;
import Utilities.Util;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot{
    
    private static Robot instance = null;
    public DriveTrain dt;
    public Navigation nav;
    public AnalogPressure airPressure;
    public ElevatorLoop lift;
    public ShooterSpeedController shooterSpeed;
    public Shooter shooter;
    public Intake intake;
    public AnalogChannel autonSelect;
    DashboardData data;
//    Compressor comp;
    public Solenoid climberTrigger;
    public MotorController humanLoader;
    public Vision vision;
    public Wrist wrist;
    private Output outputs;
    public Lights lights;
    public TurnController turn; 
    public DistanceController dist;
    private TeleController control;
    private turnThread th;
    public boolean autoTurn = false;
    private Solenoid climber;
    public boolean climberUp = false;
    private Solenoid pushrod;
    public boolean pushrodUp = false;
    
    public Robot(){
        SmartDashboard.putString("GYRO_STATUS", "NOT READY");
        lights = Lights.getInstance();
        lights.updateLights(Constants.BOOTUP);
        dt = DriveTrain.getInstance();
        nav = Navigation.getInstance();
        lift = ElevatorLoop.getInstance();
        shooterSpeed = ShooterSpeedController.getInstance();
        shooter = Shooter.getInstance();
        wrist = Wrist.getInstance();
        intake = Intake.getInstance();
//        comp = new Compressor();
        humanLoader = new MotorController(Constants.TALON_CONTROLLER, Ports.HUMANLOAD);
//       autonSelect = new AnalogChannel(Ports.AUTON_SELECT);
//        climberTrigger = new Solenoid(Ports.CLIMBER_TRIGGER);
//        climberTrigger.set(false);
//        vision = new Vision();
        airPressure = new AnalogPressure(Ports.ANALOG_PRESSURE,Constants.VOLTS_TO_PSI);
//          outputs = new Output();
//        outputs.run();
       lights.updateLights(Constants.READY);
       turn = TurnController.getInstance();
       dist = DistanceController.getInstance();
       climber = new Solenoid(Ports.CLIMBER);
       pushrod = new Solenoid(Ports.PUSHROD);
    }
    public static Robot getInstance()
    {
        if( instance == null )
            instance = new Robot();
        return instance;
    }
    public void loadParts(){
        control = TeleController.getInstance();
    }
    public void climberUp(){
        climber.set(true);
        climberUp = true;
    }
    public void climberDown(){
        climber.set(false);
        climberUp = false;
    }
    public void pushrodUp(){
        pushrod.set(true);
        pushrodUp = true;
    }
    public void pushrodDown(){
        pushrod.set(false);
        pushrodUp = false;
    }
    public void turnToHeading(double heading, double timeout){
        autoTurn = true;
        nav.resetRobotPosition(0, 0, 0,false);
        turn.reset();
        if(Math.abs(heading) <= Constants.ADJUST_TURN_DEG){
            if(heading < 0){
                turn.adjustLeft();
            }else{
                turn.adjustRight();
            }
        }else{
             turn.setGoal(heading,timeout);
        }
       /*
        while(!turn.onTarget() && !control.right.getRawButton(2) && (Util.inRange(control.left.getX(), 0.2))){
            turn.run();
        }*/
        
        th = new turnThread();
        th.run();
    }
    private class turnThread extends Thread{
        
        public void run(){
            try {
                while(!turn.onTarget() && !control.right.getRawButton(2) && (Util.inRange(control.left.getX(), 0.2))){
                    turn.run();
                }
                
                autoTurn = false;
                dt.directDrive(0, 0);
            }catch(Exception e){
                
            }
        }
    }
    
}
