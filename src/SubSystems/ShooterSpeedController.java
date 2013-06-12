package SubSystems;

import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Jared Russell
 */
public class ShooterSpeedController extends SynchronousPID implements Controller
{
    
    private double output = 0.0;
    private double kffv;

    private Shooter shooter = Shooter.getInstance();
    private boolean readyToFire = false;
    private boolean frisbeeDetected = false;
    private int numFrisbeesFired = 0;
    private static ShooterSpeedController instance = null;
    double lastOutput = 0;
    double calOutput = 0;
    double lastSpeed = 0;
    double lastTimeChecked = 0;
    boolean shotFired = false;
    
    public static ShooterSpeedController getInstance()
    {
        if( instance == null )
            instance = new ShooterSpeedController();
        return instance;
    }

    private ShooterSpeedController()
    {
        loadProperties();
    }

    public synchronized void setGoal(double rpm)
    {
        this.setSetpoint(rpm);
    }
 /*
    public synchronized void reset()
    {
        this.output = 0.0;
        this.setSetpoint(0.0);
        readyToFire = false;
        numFrisbeesFired = 0;
        frisbeeDetected = false;

        // Use the built-in SynchronousPID reset method
        super.reset();
    }
     * */
    public synchronized void run()
    {
        
        double speed = 0;
        if(this.getSetpoint() > 6000){
            shooter.set(-1);
            speed = speed = Util.buffer(shooter.getShooterSpeed(), lastSpeed, 10);
            output = calculate(speed);
            lastSpeed = speed;
            
        }else{
            
            // Add in the feedforward term
            if(this.getSetpoint() == 0){
                shooter.set(0);
                lastSpeed = 0;
            }else{
                speed = speed = Util.buffer(shooter.getShooterSpeed(), lastSpeed, 10);
                output = calculate(speed);
                lastSpeed = speed;
                calOutput = Util.limit(Math.abs(output + kffv*getSetpoint()), 0, 1);
                shooter.set(-calOutput);
                if(Util.onTarget(getSetpoint(), speed, Constants.SHOOTER_ON_TARGET_SPEED)){
                    readyToFire = true;
                    shooter.lights.updateLights(Constants.READYTOSHOOT);
                }else{
                    readyToFire = false;
                    shooter.lights.updateLights(Constants.NOTREADYTOSHOOT);
                }
                SmartDashboard.putNumber("SHOOTER_POWER", calOutput);
    //            System.out.println(" cal " + calOutput +  " output " + output + " speed " + speed + " act " + shooter.getShooterSpeed());
            }
        }

        SmartDashboard.putNumber("SHOOTER_SPEED",speed );
        SmartDashboard.putNumber("SHOOTER_SPEED2",speed );
        SmartDashboard.putNumber("SHOOTER_GOAL", this.getSetpoint());
        
        SmartDashboard.putNumber("SHOOTER_P", this.getP());
        SmartDashboard.putNumber("SHOOTER_D", this.getD());
        SmartDashboard.putBoolean("MAG STOWED", shooter.getMagInStowPosition());
    }
    public boolean encoderMalfunction(){
        if(this.getSetpoint() > 5000 && shooter.getShooterSpeed() < 10){
            return true;
        }
        return false;
    }
    public void shooterGoTo(double _goal){ this.setGoal(Util.limit(_goal, Constants.SHOOTER_MIN_RPM, Constants.SHOOTER_MAX_RPM));  } 
    public void fullCourtShot(){this.setGoal(6200); }
    public void floorShot(){ this.setGoal(5100); }
    public void bottomShot(){ this.setGoal(5100); }
    public void stop(){ this.setGoal(0);}
    public void shot(){ shotFired = true;}
    public void adjust(double accel){ this.setGoal(this.getSetpoint() + (accel * 200)); } 
    public void upPIDp(){
        double p = this.getP();
        double i = this.getI();
        double d = this.getD();
        p+= 0.001;
        setPID(p, i, d);
    }
    public void downPIDp(){
        double p = this.getP();
        double i = this.getI();
        double d = this.getD();
        p-= 0.001;
        setPID(p, i, d);
    }
    public void upPIDd(){
        double p = this.getP();
        double i = this.getI();
        double d = this.getD();
        d+= 0.001;
        setPID(p, i, d);
    }
    public void downPIDd(){
        double p = this.getP();
        double i = this.getI();
        double d = this.getD();
        d-= 0.001;
        setPID(p, i, d);
    }
    public synchronized boolean onTarget()
    {
        return readyToFire;
    }
    public synchronized boolean fastonTarget()
    {
        return shooter.getShooterSpeed() > 4500;
    }
    public synchronized boolean fastonTarget2()
    {
        return shooter.getShooterSpeed() > 4500;
    }
    public synchronized int getNumFrisbeesFired()
    {
        return numFrisbeesFired;
    }

    public final void loadProperties()
    {
        double kp = Constants.SHOOTER_KP;
        double ki = Constants.SHOOTER_KI;
        double kd = Constants.SHOOTER_KD;
        kffv = Constants.SHOOTER_KFFV;
        this.setInputRange(Constants.SHOOTER_MIN_RPM, Constants.SHOOTER_MAX_RPM);
        this.setOutputRange(-0.2, 0.2);
        setPID(kp,ki,kd);
    }
}
