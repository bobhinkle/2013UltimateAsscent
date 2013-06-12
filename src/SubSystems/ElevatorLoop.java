package SubSystems;

import Sensors.MotorController;
import Sensors.SuperEncoder;
import Team1323Robot.Robot;
import Team1323Robot.Team1323RobotFSM;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Jared Russell
 */
public class ElevatorLoop extends SynchronousPID implements Controller
{
    private MotorController lift;
    private SuperEncoder eleEnc;
    private DigitalInput limitSwitch;
    
    private double goalPosition;
    private boolean isOnTarget = false;
    private static final int onTargetThresh = 25;
    private int onTargetCounter = onTargetThresh;
    public static double kOnTargetToleranceInches = Constants.DISTANCE_TOLERANCE;
    public static final double kLoopRate = 200.0;

    private static ElevatorLoop instance = null;
    private Robot robot;
    private Team1323RobotFSM fsm;
    public static ElevatorLoop getInstance()
    {
        if( instance == null )
            instance = new ElevatorLoop();
        return instance;
    }

    private ElevatorLoop()
    {
        loadProperties();
        if(Constants.USE_TALONS){
            lift = new MotorController(Constants.TALON_CONTROLLER,Ports.ELEVATOR_LIFT);
        }else{
            lift = new MotorController(Constants.VICTOR_CONTROLLER,Ports.ELEVATOR_LIFT);
        }
        eleEnc = new SuperEncoder(Ports.LIFTENC,Ports.LIFTENC+1,false,2);
        eleEnc.setPIDReturn(1);
        eleEnc.setDistancePerPulse(Constants.ELEVATOR_DISTANCE_PER_PULSE);
        eleEnc.start();
        limitSwitch = new DigitalInput(Ports.ELELIMIT);
        
        goalPosition = eleEnc.getDistance();
        this.setInputRange(Constants.ELEVATOR_MIN_HEIGHT, Constants.ELEVATOR_MAX_HEIGHT);
        this.setOutputRange(-Math.abs(Constants.ELEVATOR_MAX_POWER), Math.abs(Constants.ELEVATOR_MAX_POWER));
    }

    public synchronized void setGoal(double goalDistance)
    {
        reset();
        this.setSetpoint(goalDistance);
        goalPosition = goalDistance;
    }
    public void stop(){ this.setGoal(this.getHeight());}
    public double getHeight(){
        return eleEnc.getDistance();
    }
    public void loadParts(){
        robot = Robot.getInstance();
        fsm = Team1323RobotFSM.getInstance();
    }
    public synchronized void reset()
    {
        super.reset();
        isOnTarget = false;
        onTargetCounter = onTargetThresh;
    }
    public boolean checkLimit(){
        if(!limitSwitch.get()){
               eleEnc.reset();
               return true;
        }else{
            return false;
        }
    }
    public void lowerP(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        
        p -= 0.01;
        this.setPID(p, i, d);
    }
    public void upP(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        
        p += 0.01;
        this.setPID(p, i, d);
    }
    public void lowerD(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        
        d -= 0.001;
        this.setPID(p, i, d);
    }
    public void upD(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        
        d += 0.001;
        this.setPID(p, i, d);
    }
    public void trim(double inches){
        reset();
        if(inches > 0){
            this.setGoal(Util.limit(this.goalPosition + inches, 0, Constants.ELEVATOR_MAX_HEIGHT));
        }else{
            this.setGoal(Util.limit(this.goalPosition + inches, Constants.ELEVATOR_MIN_HEIGHT, Constants.ELEVATOR_MAX_HEIGHT));
        }        
    }
    public synchronized void run()
    {
        checkLimit();
        double current = eleEnc.getDistance();
        double difference = this.getSetpoint() - current;
        
        double calPower = this.calculate(current);
        double power = Util.pidPower(-calPower, -Math.abs(Constants.ELEVATOR_MIN_POWER), -Math.abs(Constants.ELEVATOR_MAX_POWER),Math.abs(Constants.ELEVATOR_MIN_POWER), Math.abs(Constants.ELEVATOR_MAX_POWER));
        
        if(Util.onTarget(goalPosition,current,kOnTargetToleranceInches) || isOnTarget )
        {
            if(onTarget())
                isOnTarget = true;
            onTargetCounter--;
            if(this.getSetpoint() == 0){
                power = 0;
                System.out.println("11");
            }else{
                System.out.println("10");
//                lift.set(power);
            }
            if(this.goalPosition < 0){
                eleEnc.reset();
                this.setGoal(0);
            }
        }
        else
        {
           if(!checkLimit()){
               if(current >= Constants.ELEVATOR_MAX_HEIGHT){
                   if(power > 0){ // move down
 //                      lift.set(power);
 //                      System.out.println("6");
                   }else{
                       power = 0;
//                       System.out.println("7");
                   }
                }else{
                   if((fsm.getCurrentState() == Team1323RobotFSM.FLOOR_INTAKE) ||(fsm.getCurrentState() == Team1323RobotFSM.FLOOR_STOW)){
                       if(!Util.onTarget(robot.wrist.getAngle(), Constants.WRIST_MIN_ANGLE, 5)){
                           power = 0 ;
//                           System.out.println("2");                     
                       }
                   }
                }
           }else{
               if((difference > 0) && (power < 0)){ //only move up
 //                  lift.set(power);
 //                 System.out.println("4");
               }else{
                   power = 0;
//                   System.out.println("5");
               }
           }
            onTargetCounter = onTargetThresh;
            isOnTarget = false;
        }
        lift.set(power);
        SmartDashboard.putNumber("ELE_HEIGHT", current);
        SmartDashboard.putNumber("ELE_GOAL", goalPosition);
        SmartDashboard.putNumber("ELE_POWER", power);
        SmartDashboard.putNumber("ELE_P", this.getP());
        SmartDashboard.putNumber("ELE_I", this.getI());
        SmartDashboard.putNumber("ELE_D", this.getD());
    }

    public synchronized boolean onTarget()
    {
        return onTargetCounter <= 0;
    }
    public boolean withinRange(double current){
        if(((Math.abs(goalPosition) - Math.abs(current)) < kOnTargetToleranceInches + 0.5) &&  ((Math.abs(goalPosition) - Math.abs(current)) > -(kOnTargetToleranceInches + 0.5) )){
            return true;
        }else{
            return false;
        }
    }
    public synchronized boolean onTargetNow()
    {
        return Util.onTarget(this.getSetpoint(), eleEnc.getDistance(), Constants.ELEVATOR_TOLERANCE);
    }
    public final void loadProperties()
    {
        double kp = Constants.ELEVATOR_P;
        double ki = Constants.ELEVATOR_I;
        double kd = Constants.ELEVATOR_D;
        this.setPID(kp, ki, kd);
        
        kOnTargetToleranceInches = Constants.ELEVATOR_TOLERANCE;
    }
}
