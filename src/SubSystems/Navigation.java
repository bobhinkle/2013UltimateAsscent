package SubSystems;

import Sensors.CustomGyro;
import Sensors.SuperEncoder;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author jrussell
 */
public class Navigation implements PIDSource
{
    // Sensors
    protected SuperEncoder leftDriveEncoder;
    protected SuperEncoder rightDriveEncoder;
    protected CustomGyro gyro;

    // Navigational state
    private double x = 0.0; // positive from driver facing center of the field
    private double y = 0.0; // positive from driver looking left
    private double theta0 = 0.0; // anti-clockwise from center of field to left
    private double thetaLast = 0.0;
    private double speedX = 0;
    private double timeLast = 0;
    private static Navigation instance;
    private double basicDistance = 0;
    private double topGoalPosition = 360;
    private boolean topGoalFound = false;
    private Navigation()
    {
        leftDriveEncoder = new SuperEncoder(Ports.LEFTENC,Ports.LEFTENC+1,true,1);
        leftDriveEncoder.setDistancePerPulse(Constants.DRIVE_DISTANCE_PER_PULSE);
        leftDriveEncoder.start();
        rightDriveEncoder = new SuperEncoder(Ports.RIGHTENC,Ports.RIGHTENC+1,false,1);
        rightDriveEncoder.setDistancePerPulse(Constants.DRIVE_DISTANCE_PER_PULSE);
        rightDriveEncoder.start();
        SmartDashboard.putString("GYRO_STATUS", "INITIALIZING");
        gyro = new CustomGyro(Ports.GYRO);
        SmartDashboard.putString("GYRO_STATUS", "READY");
    }

    public static Navigation getInstance()
    {
        if( instance == null )
        {
            instance = new Navigation();
        }
        return instance;
    }

    public synchronized void resetRobotPosition(double x, double y, double theta,boolean gyroReset)
    {
        this.x = x;
        this.y = y;
        theta0 = theta;
        thetaLast = theta;
        leftDriveEncoder.reset();
        rightDriveEncoder.reset();
        if(gyroReset)
            gyro.reset();
        basicDistance = 0;
    }
    public void checkForTopGoal(){
        if(SmartDashboard.getBoolean("found", false)){
            double angle = SmartDashboard.getNumber("azimuth", 0);
            if((340 > angle && angle < 360) || (0 > angle && angle < 5)){
                if(Util.getDifferenceInAngleDegrees(angle, SmartDashboard.getNumber("azimuth", 0)) < Util.getDifferenceInAngleDegrees(topGoalPosition, SmartDashboard.getNumber("azimuth", 0))){
                    topGoalPosition = SmartDashboard.getNumber("azimuth", 0);
                }
            }else{
                topGoalFound = false;
            }
        }
    }
    public boolean topGoalFound(){
        return topGoalFound;
    }
    public double topGoalAngle(){
        return Util.getDifferenceInAngleDegrees(topGoalPosition,getHeadingInDegrees());
    }
    public synchronized double getX()
    {
        return x;
    }

    public synchronized double getY()
    {
        return y;
    }

    public double getHeadingInDegrees()
    {
        return Util.boundAngle0to360Degrees(gyro.getAngle());
        
    }
    public double getRawHeading(){
        return gyro.getAngle();
    }

    public double getPitchInDegrees()
    {
        //return gyro.getAngle();
        return 0;
    }

    public void resetPitch()
    {
        gyro.reset();
    }

    public synchronized void run()
    {
        updatePosition();
        checkForTopGoal();
        SmartDashboard.putNumber("Distance",getDistance());
        SmartDashboard.putNumber("Heading",getHeadingInDegrees());
        SmartDashboard.putNumber("TOP_GOAL", topGoalPosition);
        SmartDashboard.putNumber("TOP_GOAL)DIFF", topGoalAngle());
    }

    public double getLeftEncoderDistance()
    {
        return leftDriveEncoder.getDistance();
    }

    public double getRightEncoderDistance()
    {
        return rightDriveEncoder.getDistance();
    }
    public double getDistance(){
        return basicDistance;
    }
    public void updatePosition()
    {
//        basicDistance = (leftDriveEncoder.getDistance() + rightDriveEncoder.getDistance())/2.0;
        basicDistance = leftDriveEncoder.getDistance();
        /*
        double distanceTravelled = ((leftDriveEncoder.getDistance() + rightDriveEncoder.getDistance())/2.0) - distanceLast;
        double timePassed = System.currentTimeMillis() - timeLast;
        speedX = distanceTravelled/timePassed;
        x += distanceTravelled * Math.cos(Math.toRadians(getHeadingInDegrees()));
        y += distanceTravelled * Math.sin(Math.toRadians(getHeadingInDegrees()));

        distanceLast = (leftDriveEncoder.getDistance() + rightDriveEncoder.getDistance())/2.0;
        timeLast = System.currentTimeMillis();*/
    }
    public double pidGet() {
        return basicDistance;
    }
    
    public class Distance implements PIDSource {
    
        public double pidGet(){
            return basicDistance = (leftDriveEncoder.getDistance() + rightDriveEncoder.getDistance())/2.0;
        }
    }
}
