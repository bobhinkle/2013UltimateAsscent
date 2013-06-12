package SubSystems;

import Team1323Robot.Robot;
import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Jared Russell
 */
public class DistanceController extends SynchronousPID implements Controller
{
    private Robot robot;
    private SynchronousPID straightController;

    private double goalPosition;
    private double maxVelocity;
    private boolean isOnTarget = false;
    private static final int onTargetThresh = 5;
    private int onTargetCounter = onTargetThresh;
    public static double kOnTargetToleranceInches = Constants.DISTANCE_TOLERANCE;
    public static final double kLoopRate = 200.0;
    private double timeout = 0;
    private double startTime = 0;
    private double heading = 0;
    private double tempTol = 4;
    private static DistanceController instance = null;
    private boolean holdSpeed = false;

    public static DistanceController getInstance()
    {
        if( instance == null )
            instance = new DistanceController();
        return instance;
    }
    public void loadParts(){
        robot = Robot.getInstance();
    }
    private DistanceController()
    {
        loadProperties();
    }

    public synchronized void setGoal(double goalDistance, double maxPower, double angle, double time, double tol,boolean force)
    {
        this.setSetpoint(goalDistance);
        this.maxVelocity = maxPower;
        this.setOutputRange(-Math.abs(maxPower), Math.abs(maxPower));
        goalPosition = goalDistance;
        startTime = System.currentTimeMillis();
        timeout = (1000 * time) + startTime;
        heading = angle;
        tempTol = tol;
        setTime();
        holdSpeed = force;
    }
    double lastDistance = 0;
    double lastCheck = 0;
    public void setTime(){
        lastCheck = System.currentTimeMillis();
    }
    public boolean checkStall(){
        double distanceTraveled = Math.abs(robot.nav.getDistance()) - Math.abs(lastDistance);
        double timeSinceCheck = System.currentTimeMillis() - lastCheck;
        if((distanceTraveled < 0.2) && (timeSinceCheck > 1)){
            return true;
        }
        System.out.println("CS: " + distanceTraveled + " " + timeSinceCheck);
        lastDistance = robot.nav.getDistance();
        lastCheck = System.currentTimeMillis();
        return false;
    }

    public synchronized void reset()
    {
        super.reset();
        straightController.reset();
//        robot.nav.resetRobotPosition(0.0, 0.0, 0.0,false);
        isOnTarget = false;
        onTargetCounter = onTargetThresh;
        heading = 0;
    }
    public synchronized void resetDistance(){
        robot.nav.resetRobotPosition(0.0, 0.0, 0.0,false);
    }
    public synchronized void run()
    {
        robot.nav.run();
        double current = robot.nav.getDistance();
        double power = this.calculate(current);
        double turn = straightController.calculate(Util.getDifferenceInAngleDegrees(heading, robot.nav.getHeadingInDegrees()));
        if(holdSpeed){
            if(inRange(current)){
                isOnTarget = true;
                robot.dt.directDrive(0, 0);
                onTargetCounter = -1;
            }else{
                if(this.getSetpoint() < 0){
                    robot.dt.cheesyDrive(turn, -this.maxVelocity, false);
                }else{
                    robot.dt.cheesyDrive(turn, this.maxVelocity, false);
                }
//                System.out.println("HS " + current + " " + this.getSetpoint());
            }
        }else{
            if(inRange(current))
            {
                if(onTarget() || checkStall() || outOfTime()){
                    isOnTarget = true;
                }
                onTargetCounter--;
                robot.dt.cheesyDrive(turn, power, false);
                System.out.println("S1" + current + " " + this.getSetpoint() + " " + onTarget());
            }
            else
            { 
                onTargetCounter = onTargetThresh;
                isOnTarget = false;
                robot.dt.cheesyDrive(turn, power, false);
                System.out.println("S2" + current + " " + this.getSetpoint());
            }
        }
        SmartDashboard.putNumber("DIST_POWER", power);
    }

    public boolean inRange(double current){ return Util.onTarget(goalPosition,current, tempTol);}
    public synchronized boolean onTarget()
    {
        return onTargetCounter <= 0;
    }

    public boolean outOfTime(){
        if(timeout < System.currentTimeMillis()){
            return true;
        }else{
            return false;
        }
    }
    public final void loadProperties()
    {
        double kp = Constants.DIST_KP;
        double ki = Constants.DIST_KI;
        double kd = Constants.DIST_KD;
        this.setPID(kp, ki, kd);
        this.setOutputRange(-1, 1);

        kp = Constants.STRAIGHT_KP;
        ki = Constants.STRAIGHT_KI;
        kd = Constants.STRAIGHT_KD;
        straightController = new SynchronousPID(kp,ki,kd);

        kOnTargetToleranceInches = Constants.DISTANCE_TOLERANCE;
    }
}
