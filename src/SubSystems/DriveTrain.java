package SubSystems;

import Sensors.MotorController;
import Utilities.Constants;
import Utilities.Ports;
import Utilities.Util;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveTrain
{
    public MotorController leftdt, rightdt;
    private Solenoid shifter;
    private double currentSpeed = 0; 
    private double innerSpeed   = 0;
    private static final double DRIVE_TRAIN_DEADBAND = 0.15;
    public boolean gear = Constants.HIGH_GEAR;
    
    //Constants for carDrive
    public static double RADIUS_MOD = 1;
    public static double ROBOT_WIDTH = 1;
    public static double MAX_DIAMETER  = 1.5;   //Largest Turning Radius 15ft
    double maxLowTurnSpeed =0.55;
    double maxHighTurnSpeed = 0.9;
    public double thrustOut;
    public double rotOut;
    
    private static DriveTrain instance = null;
    public DriveTrain()
    {
        /*
        if(Constants.USE_TALONS){
            leftdt  = new MotorController(Constants.TALON_CONTROLLER,Ports.LEFTDT);
            rightdt = new MotorController(Constants.TALON_CONTROLLER,Ports.RIGHTDT);      
        }else{
            leftdt  = new MotorController(Constants.VICTOR_CONTROLLER,Ports.LEFTDT);
            rightdt = new MotorController(Constants.VICTOR_CONTROLLER,Ports.RIGHTDT); 
        }*/
        leftdt  = new MotorController(Constants.TALON_CONTROLLER,Ports.LEFTDT);
        rightdt = new MotorController(Constants.TALON_CONTROLLER,Ports.RIGHTDT);
        shifter = new Solenoid(Ports.SHIFTER);
    }
    public static DriveTrain getInstance()
    {
        if( instance == null )
            instance = new DriveTrain();
        return instance;
    }
    public void directDrive(double left, double right)
    {
        leftdt.set(left);
        rightdt.set(-right);
    }
    public void directArcadeDrive(double x, double y)
    {
        x = Util.limit(x, -1.0, 1.0);
        y = Util.limit(y, -1.0, 1.0);
        double left = y + x;
        double right = y - x;
        left = Util.limit(left, -1.0, 1.0);
        right = Util.limit(right, -1.0, 1.0);
        directDrive(left, right);
    }
    public void driveSpeedTurn(double speed, double turn)
    {
        double left = speed + turn;
        double right = speed - turn;
        directDrive(left, right);
    }
    public void carDrive2(double wheel, double power){
        
        if (power > -DRIVE_TRAIN_DEADBAND && power < DRIVE_TRAIN_DEADBAND)
            power = 0.0;
        if (wheel > -DRIVE_TRAIN_DEADBAND && wheel < DRIVE_TRAIN_DEADBAND)
            wheel = 0.0;
        if( (wheel == 0) && (power == 0)){
            currentSpeed = 0;
            innerSpeed = 0;
        }else{
            currentSpeed = (((currentSpeed *9) + power)/10);
            innerSpeed = (1-Math.abs(wheel)*1.2) * currentSpeed;
        }
        
        if((power == 0) && ((wheel < -0.5) || (wheel > 0.5))){

            if(gear == Constants.LOW_GEAR){
                directDrive(wheel - 0.5,-(wheel - 0.5));
            }else{
                directDrive(wheel - 0.3,-(wheel - 0.3));
            }
        }else if(power > 0){
            if(wheel > 0){
                directDrive(currentSpeed,innerSpeed);
            }else{
                directDrive(innerSpeed,currentSpeed);
            }
        }else{
            if(wheel > 0){
                directDrive(innerSpeed,currentSpeed);
            }else{
                directDrive(currentSpeed,innerSpeed);
            }
        }
        
        
    }
    public void carDrive(double wheel, double thrust)
    {
       
        if (thrust > -DRIVE_TRAIN_DEADBAND && thrust < DRIVE_TRAIN_DEADBAND)
            thrust = 0.0;

        if (wheel > -DRIVE_TRAIN_DEADBAND && wheel < DRIVE_TRAIN_DEADBAND)
            wheel = 0.0;
        double RadiusOutter = (MAX_DIAMETER / 2 ) * (wheel);
        double RadiusInner = ((MAX_DIAMETER-ROBOT_WIDTH   ) / 2) * (wheel);
        double SpeedInner;
        if(wheel == 0){
            SpeedInner = thrust;
        }else{
             SpeedInner = thrust * (RadiusInner / RadiusOutter) ;
        }
        double power;
        double trueSpeed;
        
        power = SpeedInner;
        trueSpeed = thrust;
        
        if (power > 1.0) {
                trueSpeed -= (power - 1.0);
                power = 1.0;
        } 
        if (trueSpeed > 1.0) {
                power -= (trueSpeed - 1.0);
                trueSpeed = 1.0;
        } 
        if (power < -1.0) {
                trueSpeed += (-1.0 - power);
                power = -1.0;
        } 
        if (trueSpeed < -1.0) {
                power += (-1.0 - trueSpeed);
                trueSpeed = -1.0;
        }
        
        if((thrust == 0) && ((wheel < -0.52) || (wheel > 0.52))){
            power = wheel;
            directDrive(power,-power);
        }else if(thrust > 0){
            if(wheel > 0){
                directDrive(-power,-trueSpeed);
            }else{
                directDrive(-trueSpeed,-power);
            }
        }else{
            if(wheel > 0){
                directDrive(-trueSpeed,-power);
            }else{
                directDrive(-power,-trueSpeed);
            }
        }
//        System.out.println("Wheel "+wheel+ " thrust "+thrust +
//                " SpeedI " + SpeedInner + " RadO " + RadiusOutter + " RadI " + RadiusInner + " Power: " + power);
    }

    public void setGear(boolean gear) {this.gear = gear; shifter.set(gear); }
    public void lowGear(){ shifter.set(Constants.LOW_GEAR);}
    public void highGear(){ shifter.set(Constants.HIGH_GEAR);}
    public boolean getGear(){ return shifter.get(); }
    public boolean inLowGear() { return shifter.get() == Constants.LOW_GEAR;}
    private double old_wheel = 0.0;
    private double neg_inertia_accumulator = 0.0;
    public void cheesyDrive(double wheel, double throttle, boolean quickturn)
    {
        double left_pwm,right_pwm,overPower;
        double sensitivity = 1.1;
        double angular_power;
        double linear_power;
        double wheelNonLinearity;

        double neg_inertia = wheel - old_wheel;
        old_wheel = wheel;

        if (!inLowGear()) {
                wheelNonLinearity = 0.995; // used to be csvReader->TURN_NONL 0.9
                // Apply a sin function that's scaled to make it feel bette
                wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
                wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
        } else {
                wheelNonLinearity = 0.8; // used to be csvReader->TURN_NONL higher is less sensitive
                // Apply a sin function that's scaled to make it feel bette
                wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
                wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
                wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
        }

        double neg_inertia_scalar;
        if (!inLowGear()) {
                neg_inertia_scalar = 1.25; // used to be csvReader->NEG_INT 11
                sensitivity = 1.06; // used to be csvReader->SENSE_HIGH  1.15
                if (Math.abs(throttle) > 0.1) { // used to be csvReader->SENSE_
                        sensitivity = .9 - (.9 - sensitivity) / Math.abs(throttle);
                }
        } else {
                if (wheel * neg_inertia > 0) {
                        neg_inertia_scalar = 1; // used to be csvReader->NE 5
                } else {
                        if (Math.abs(wheel) > 0.65) {
                                neg_inertia_scalar = 1;// used to be csvRe 10
                        } else {
                                neg_inertia_scalar = 1; // used to be csvRe 3
                        }
                }
                sensitivity = 1.07; // used to be csvReader->SENSE_LOW lower is less sensitive

                if (Math.abs(throttle) > 0.1) { // used to be csvReader->SENSE_
                        sensitivity = .9 - (.9 - sensitivity) / Math.abs(throttle);
                }
        }
        double neg_inertia_power = neg_inertia * neg_inertia_scalar;
        if (Math.abs(throttle) >= 0.05 || quickturn) neg_inertia_accumulator += neg_inertia_power;
        
        wheel = wheel + neg_inertia_accumulator;
        if (neg_inertia_accumulator > 1)
                neg_inertia_accumulator -= 0.25;
        else if (neg_inertia_accumulator < -1)
                neg_inertia_accumulator += 0.25;
        else
                neg_inertia_accumulator = 0;

        linear_power = throttle;

        if ((!Util.inRange(throttle, -0.15,0.15) || !(Util.inRange(wheel, -0.4, 0.4))) && quickturn) {
                overPower = 1.0;
                if (gear == Constants.HIGH_GEAR) {
                        sensitivity = 1.0; // default 1.0
                } else {
                        sensitivity = 1.0;
                }
                angular_power = wheel * sensitivity;
        } else {
                overPower = 0.0;
                angular_power = Math.abs(throttle) * wheel * sensitivity;
        }
//        System.out.println("NA " + neg_inertia_accumulator + " AP " + angular_power + " wheel " + wheel + " throttle" + throttle + " NAP " + neg_inertia_power);
        right_pwm = left_pwm = linear_power;
        left_pwm += angular_power;
        right_pwm -= angular_power;

        if (left_pwm > 1.0) {
                right_pwm -= overPower*(left_pwm - 1.0);
                left_pwm = 1.0;
        } else if (right_pwm > 1.0) {
                left_pwm -= overPower*(right_pwm - 1.0);
                right_pwm = 1.0;
        } else if (left_pwm < -1.0) {
                right_pwm += overPower*(-1.0 - left_pwm);
                left_pwm = -1.0;
        } else if (right_pwm < -1.0) {
                left_pwm += overPower*(-1.0 - right_pwm);
                right_pwm = -1.0;
        }
//        directDrive(Util.deadBand(Util.victorLinearize(left_pwm),DRIVE_TRAIN_DEADBAND),Util.deadBand(Util.victorLinearize(right_pwm),DRIVE_TRAIN_DEADBAND));
        directDrive(Util.victorLinearize(left_pwm),Util.victorLinearize(right_pwm));
    }
    
    double powerToReduce = 0.0;
    int lastDirection    = 0;
    public void driveHoldHeading(double headingToHold, double currentHeading,double maxSpeed){
        if(currentHeading < headingToHold){
            if(lastDirection != 1)
                powerToReduce = 0;
            if((Math.abs(maxSpeed) - powerToReduce) > 0)
                powerToReduce = powerToReduce + 0.05;
            SmartDashboard.putString("driveHolding", "turn right");
            lastDirection = 1;
            directArcadeDrive(maxSpeed  , maxSpeed - powerToReduce);
        }else if(currentHeading > headingToHold){
            if(lastDirection != -1)
                powerToReduce = 0;
            if((Math.abs(maxSpeed) - powerToReduce) > 0)
                powerToReduce = powerToReduce + 0.05;
            directArcadeDrive(maxSpeed - powerToReduce, maxSpeed );
            lastDirection = -1;
            SmartDashboard.putString("driveHolding", "turn left");
        }else{
            powerToReduce = 0.0;
            SmartDashboard.putString("driveHolding", "straight");
            lastDirection = 0;
            directArcadeDrive(maxSpeed, maxSpeed);
        }
        SmartDashboard.putNumber("POWER_TO_REDUCE", powerToReduce);
        SmartDashboard.putNumber("POWER_REDUCTION",maxSpeed-powerToReduce);
    }
    
    
}
