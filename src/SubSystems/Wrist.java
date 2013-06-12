package SubSystems;

import Sensors.MA3;
import Sensors.MotorController;
import Team1323Robot.Robot;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Jared Russell
 */
public class Wrist extends SynchronousPID implements Controller
{
    private MA3 armEncoder;
    private MotorController drive;
    private Robot robot;
    private double goalPosition;
    private boolean isOnTarget = false;
    private static final int onTargetThresh = 25;
    private int onTargetCounter = onTargetThresh;
    public static double kOnTargetToleranceInches = Constants.WRIST_TOLERANCE;
    public static final double kLoopRate = 200.0;
    double lastValue = 0;
    private static Wrist instance = null;
    private boolean setPlunger = false;
    public static Wrist getInstance()
    {
        if( instance == null )
            instance = new Wrist();
        return instance;
    }
    public void upP(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
         
        p += 0.01;
        this.setPID(p, i, d);
        SmartDashboard.putNumber("TURN_P", p);
        SmartDashboard.putNumber("TURN_I", i);
        SmartDashboard.putNumber("TURN_D", d);
        System.out.println(p);
    }
    public void downP(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        p -= 0.01;
        this.setPID(p, i, d);
        SmartDashboard.putNumber("TURN_P", p);
        SmartDashboard.putNumber("TURN_I", i);
        SmartDashboard.putNumber("TURN_D", d);
        System.out.println(p);
    }
    public void upD(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        d += 0.01;
        this.setPID(p, i, d);
        SmartDashboard.putNumber("TURN_P", p);
        SmartDashboard.putNumber("TURN_I", i);
        SmartDashboard.putNumber("TURN_D", d);
        System.out.println(d);
    }
    public void downD(){
        double p = this.getP();
        double d = this.getD();
        double i = this.getI();
        d -= 0.01;
        this.setPID(p, i, d);
        SmartDashboard.putNumber("TURN_P", p);
        SmartDashboard.putNumber("TURN_I", i);
        SmartDashboard.putNumber("TURN_D", d);
        System.out.println(d);
    }

    private Wrist()
    {
        loadProperties();
        armEncoder = new MA3(Ports.SHOOTER_ARM);
        /*
        if(Constants.USE_TALONS){
            drive = new MotorController(Constants.TALON_CONTROLLER,Ports.SHOOTER_WRIST_MOTOR);
        }else{
            drive = new MotorController(Constants.VICTOR_CONTROLLER,Ports.SHOOTER_WRIST_MOTOR);
        }*/
        drive = new MotorController(Constants.TALON_CONTROLLER,Ports.SHOOTER_WRIST_MOTOR);
        setGoal(armEncoder.getAngle());
    }

    public synchronized void setGoal(double goalDistance)
    {
        reset();
        this.setSetpoint(goalDistance);
        goalPosition = goalDistance;
    }
    public void loadParts(){
        
        robot = Robot.getInstance();
    }
    public double getAngle(){
        double current = armEncoder.getAngle();
        if(current < Constants.WRIST_MIN_ANGLE){
            current = Constants.WRIST_MIN_ANGLE;
        }
        return current;
    }
    public double getCurrent(){
        return lastValue;
    }
    public synchronized void reset()
    {
        super.reset();
        isOnTarget = false;
        onTargetCounter = onTargetThresh;
    }
    public void stop(){this.setGoal(getAngle());}
    private boolean inRange(double current){
        if(Util.onTarget(this.getSetpoint(),current,kOnTargetToleranceInches)){
            return true;
        }
        return false;
    }
    public void setPlunger(){
        setPlunger = true;
    }
    public synchronized void run()
    {
        double current = 0;
        double power = 0;
        if(Math.abs(this.getSetpoint() - armEncoder.getAngle()) < 1.0){
            current = Util.buffer(getAngle(), lastValue, 80);
        }else{
            current = getAngle();
        }
        if(current < Constants.WRIST_MIN_ANGLE){
            current = Constants.WRIST_MIN_ANGLE;
        }
        lastValue = current;
        
        if(setPlunger && (current > Constants.WRIST_PLUNGER_ANGLE)){
            robot.shooter.triggerOut();
            setPlunger = false;
        }
        SmartDashboard.putNumber("WRIST_ANGLE", current - Constants.WRIST_MIN_ANGLE);
        SmartDashboard.putNumber("WRIST_NO_FILTER",armEncoder.getAngle());
        SmartDashboard.putNumber("WRIST_GOAL",  this.getSetpoint() - Constants.WRIST_MIN_ANGLE);
        if(inRange(current))
        {
            if(onTarget())
                isOnTarget = true;
            onTargetCounter--;
            SmartDashboard.putString("WRIST_STATE", "ON POINT");
        }
        else
        {
            SmartDashboard.putString("WRIST_STATE", "1 POINT");
           power = this.calculate(current);
           if(this.getSetpoint() == Constants.WRIST_MIN_ANGLE && current <= Constants.WRIST_MIN_ANGLE){
               power = 0;
               SmartDashboard.putString("WRIST_STATE", "MIN POINT");
           }else if((current >= Constants.WRIST_MAX_ANGLE) && power < 0){
//                power = 0;
                SmartDashboard.putString("WRIST_STATE", "TOO_HIGH");
            }else{
                double difference = this.getSetpoint() - current;
                if(difference > 0){
                    if((robot.lift.getHeight() < Constants.ELEVATOR_MIN_SHOOTER_LIFT_HEIGHT) && (current >= Constants.WRIST_MAX_ANGLE_FOR_STOWED_ELEVATOR)){
                       power = 0;
                       SmartDashboard.putString("WRIST_STATE", "ELE_TOO_LOW");
                    }
                }
            }
            onTargetCounter = onTargetThresh;
            isOnTarget = false;
        }
      drive.set(-power);
      SmartDashboard.putNumber("WRIST_POWR", -power);
    }
    public boolean elevatorAboveHeight(){
        return robot.lift.getHeight() >= Constants.ELEVATOR_MIN_SHOOTER_LIFT_HEIGHT;
    }
    public void trim(double angle){
            this.setGoal(Util.limit(this.goalPosition + angle, Constants.WRIST_MIN_ANGLE, Constants.WRIST_MAX_ANGLE));
        }
    public synchronized boolean onTarget()
    {
        return onTargetCounter <= 0;
    }
    public double error(){
        return lastValue - this.getSetpoint();
    }
    public synchronized boolean onTargetNow(){
        return Util.onTarget(this.getSetpoint(), lastValue, Constants.WRIST_TOLERANCE + 0.4) || onTarget();
    }
    public final void loadProperties()
    {
        double kp = Constants.WRIST_P;
        double ki = Constants.WRIST_I;
        double kd = Constants.WRIST_D;
        this.setInputRange(Constants.WRIST_MIN_ANGLE, Constants.WRIST_MAX_ANGLE);
        this.setOutputRange(-Constants.WRIST_MAX_POWER, Constants.WRIST_MAX_POWER);
        this.setPID(kp, ki, kd);
        kOnTargetToleranceInches = Constants.WRIST_TOLERANCE;
    }
}
