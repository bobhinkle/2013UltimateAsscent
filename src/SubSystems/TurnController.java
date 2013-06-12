package SubSystems;

import Team1323Robot.Robot;
import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Jared Russell
 */
public class TurnController extends SynchronousPID implements Controller
{
    private Robot robot;
    private double goalPosition;
    private boolean isOnTarget = false;
    private static final int onTargetThresh = 25;
    private int onTargetCounter = onTargetThresh;
    public static double kOnTargetToleranceDegrees = Constants.TURN_ON_TARGET_DEG;
    public static final double kLoopRate = 200.0;
    private double timeout = 0;
    private double startTime = 0;
    private double lastHeading = 0;
    private static TurnController instance = null;
    private double maxPower = 0.75;
    private double lowGearMinPower = 0.2;
    private double lowGearMaxPower = 0.35;
    private boolean force = false;
    public static TurnController getInstance()
    {
        if( instance == null )
            instance = new TurnController();
        return instance;
    }
    public void loadParts(){
        robot = Robot.getInstance();
    }
    private TurnController()
    {
        loadProperties();
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
    double lastDistance = 0;
    double lastCheck = 0;
    public void setTime(){
        lastCheck = System.currentTimeMillis();
    }
    public boolean checkStall(){
        double distanceTraveled = Math.abs(robot.nav.getDistance()) - Math.abs(lastDistance);
        double timeSinceCheck = System.currentTimeMillis() - lastCheck;
        if((distanceTraveled < 0.2) && (timeSinceCheck > 10)){
            lastDistance = distanceTraveled;
            lastCheck = System.currentTimeMillis();
            return true;
        }
//        System.out.println("CS: " + distanceTraveled + " " + timeSinceCheck);
        lastDistance = distanceTraveled;
        lastCheck = System.currentTimeMillis();
        return false;
    }
    public synchronized void setGoal(double goalAngle, double timeout)
    {
        if(Math.abs(robot.nav.getRawHeading() - goalAngle) < 15){
            if(!robot.dt.inLowGear()){
                this.setPID(Constants.TURN_KP_2, Constants.TURN_KI_2, Constants.TURN_KD_2);
            }else{
                this.setPID(Constants.TURN_KP_LOW_L, Constants.TURN_KI_3, Constants.TURN_KD_3);
            }
        }else{
            if(!robot.dt.inLowGear()){
                this.setPID(Constants.TURN_KP, Constants.TURN_KI, Constants.TURN_KD);
            }else{
                this.setPID(Constants.TURN_KP_LOW_L, Constants.TURN_KI_3, Constants.TURN_KD_3);
            }
        }
        this.setSetpoint(goalAngle);
        goalPosition = goalAngle;
        startTime = System.currentTimeMillis();
        this.timeout = (timeout * 1000) + System.currentTimeMillis() ;
        force = false;
        setTime();
    }
    public void adjustLeft(){
        robot.nav.resetRobotPosition(0.0, 0.0, 0.0,true);
        if(robot.dt.inLowGear()){
            this.setPID(Constants.TURN_KP_LOW_L, Constants.TURN_KI_3, Constants.TURN_KD_3);
        }else{
            this.setPID(Constants.TURN_KP_3, Constants.TURN_KI_3, Constants.TURN_KD_3);
        }
        this.setSetpoint(-Constants.ADJUST_TURN_DEG);
        goalPosition = -Constants.ADJUST_TURN_DEG;
        startTime = System.currentTimeMillis();
        this.timeout = (5 * 1000) + System.currentTimeMillis() ;
        force = true;
        setTime();
    } 
    public void adjustRight(){
        robot.nav.resetRobotPosition(0.0, 0.0, 0.0,true);
        if(robot.dt.inLowGear()){
            this.setPID(Constants.TURN_KP_LOW_R, Constants.TURN_KI_3, Constants.TURN_KD_3);
        }else{
            this.setPID(Constants.TURN_KP_3, Constants.TURN_KI_3, Constants.TURN_KD_3);
        }
        this.setSetpoint(Constants.ADJUST_TURN_DEG);
        goalPosition = Constants.ADJUST_TURN_DEG;
        startTime = System.currentTimeMillis();
        this.timeout = (5 * 1000) + System.currentTimeMillis() ;
        force = true;
        setTime();
    }
    public synchronized void reset()
    {
        super.reset();
        robot.nav.resetRobotPosition(0.0, 0.0, 0.0,false);
        isOnTarget = false;
        onTargetCounter = onTargetThresh;
    }

    public synchronized void run()
    {
        robot.nav.run();
        double current = robot.nav.getRawHeading();
        double output = this.calculate(current);
        if(timeout > System.currentTimeMillis() && !Util.onTarget(this.getSetpoint(),current,kOnTargetToleranceDegrees))
        {
            if(robot.dt.inLowGear()){
                output = Util.pidPower(output, -lowGearMinPower, -lowGearMaxPower, lowGearMinPower, lowGearMaxPower);
            }else{
//                output = Util.pidPower(output, -minPower, -maxPower, minPower, maxPower);
            }
            robot.dt.directDrive(output, -output);
            onTargetCounter = onTargetThresh;
            isOnTarget = false;
            SmartDashboard.putNumber("turnPower", output);
            getError();
            
        }
        else
        {
            if(onTarget() || checkStall()){
                isOnTarget = true;
            }
            onTargetCounter--; 
        }
//        System.out.println("TS: " + " " + this.getSetpoint() + " " + current);
        lastHeading = current;
    }

    public synchronized boolean onTarget()
    {
        return onTargetCounter <= 0 ;
    }
    public double getError(){
        SmartDashboard.putNumber("error", lastHeading - this.getSetpoint());
        return lastHeading - this.getSetpoint();
    }
    public double getAbsError()
    {
        double absError = Math.abs(lastHeading - this.getSetpoint());
        SmartDashboard.putNumber("ABS_ERROR", absError);
        return absError;
    }
    public final void loadProperties()
    {
        double kp = Constants.TURN_KP;
        double ki = Constants.TURN_KI;
        double kd = Constants.TURN_KD;
        this.setPID(kp, ki, kd);
        this.setOutputRange(-this.maxPower, this.maxPower);
        this.setInputRange(-360, 360);
    }
}
