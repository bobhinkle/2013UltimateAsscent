package IO;

import Team1323Robot.Robot;
import Team1323Robot.Team1323RobotFSM;
import Utilities.Constants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Handles the input from an xbox controller in order to calculate what the
 *  forebar angles and claw state should be. It is designed to keep the logic for
 *  deciding what to do apart from the logic for how to do it
 *
 * @author Robotics
 */ 
public class TeleController
{
    public static final double STICK_DEAD_BAND = 0.1;

    private Xbox xbox,tuner;
    public Joystick right;
    public ThrustMasterWheel left;
    private Robot robot;
    private Team1323RobotFSM fsm;
    public boolean rightStickXHold = false;
    public boolean leftTriggerButtonHold = false;
    private boolean wristTrimTriggered = false;
    private boolean liftTrimTriggered = false;
    private boolean rightCenterClick = false;
    private static TeleController instance = null;
    public TeleController(){
        
        left  = new ThrustMasterWheel(1);
        right = new Joystick(2);
        xbox  = new Xbox(3);
        
        if(Constants.TUNING_MODE){
            tuner = new Xbox(4);
        }
    }
    public static TeleController getInstance(){
        if(instance == null){
            instance = new TeleController();
        }
        return instance;
    }
    public void loadParts(){
        robot = Robot.getInstance();
        fsm = Team1323RobotFSM.getInstance();
    }
    
    public void coPilot(){
        if(xbox.getAButton()){
            //mag and elevator bottom load
            fsm.setGoalState(Team1323RobotFSM.FLOOR_INTAKE);
        }
        //////////////////////////////////////////
        if(xbox.getBButton()){
            //Human player load height
            fsm.setGoalState(Team1323RobotFSM.FLOOR_STOW);
            robot.intake.allStop();
        }
        ////////////////////////////////////////
        if(xbox.getXButton()){
            //front of pyramid shot
            
            fsm.setGoalState(Team1323RobotFSM.SHOOT_PYRAMID_UNDER);
        }
        ///////////////////////////////////////
        if(xbox.getYButton()){
            //bottom of pyramid shot
            fsm.setGoalState(Team1323RobotFSM.SHOOT_PYRAMID_SIDESHOT);
        }
        /////////////////////////////////////////////

        if(xbox.getRightTrigger()){ //shoot
            if(robot.shooterSpeed.fastonTarget() || robot.shooterSpeed.encoderMalfunction()){
                robot.shooter.shoot();
                robot.shooterSpeed.shot();
            }
        }
        ///////////////////////////////////////////////////////
        if(xbox.getLeftTrigger()){
           //ready to shoot a shot
            /*
            if(fsm.getCurrentState() == Team1323RobotFSM.SHOOT_PYRAMID_CENTERLINE){
                robot.shooterSpeed.bottomShot();
            }else if(fsm.getCurrentState() == Team1323RobotFSM.SHOOT_PYRAMID_SIDESHOT){
                robot.shooterSpeed.floorShot();
            }else if(fsm.getCurrentState() == Team1323RobotFSM.HUMAN_LOAD){
                robot.shooterSpeed.fullCourtShot();
                robot.wrist.setGoal(Constants.WRIST_FULL_FIELD_ANGLE);
            }
            else{
                robot.shooterSpeed.floorShot();
            }*/
            robot.shooterSpeed.fullCourtShot();            
        }
        //////////////////////////////////////////////////////////////////// 
        if(xbox.getLeftBumper()){ //reverse intake roller
            if ((fsm.getCurrentState() == Team1323RobotFSM.FLOOR_STOW) || (fsm.getCurrentState() == Team1323RobotFSM.HUMAN_LOAD)) {
                robot.humanLoader.set(1.0);
            }
            else {
                robot.intake.allReverse();
            }
        }
        //////////////////////////////////
        if(xbox.getRightBumper()) {
            if ((fsm.getCurrentState() == Team1323RobotFSM.FLOOR_STOW) || (fsm.getCurrentState() == Team1323RobotFSM.HUMAN_LOAD)) {
                robot.humanLoader.set(-1.0);
            } else {
                if(!robot.shooter.getMagInStowPosition()){
                    robot.lights.updateLights(Constants.INTAKEON);
                    robot.intake.allForward();
                }
            }
        }
        //////////////////////////////////////////////////////



        if(xbox.getBackButton()){  // stop all 
           robot.intake.allStop(); 
           robot.shooterSpeed.setGoal(0);
           robot.humanLoader.set(0);
           robot.lights.updateLights(Constants.READY);
           fsm.setGoalState(Team1323RobotFSM.INIT);
//           robot.shooter.triggerIn();
        }
        ////////////////////////////////////////////////////////
        if(xbox.getStartButton()){
            fsm.setGoalState(Team1323RobotFSM.HUMAN_LOAD);
        }

        ////////////////////////////////////////////
        if (xbox.getLeftStickY() > 0.4) {
            if (!liftTrimTriggered) {
                robot.lift.trim(-1.0);
                liftTrimTriggered = true;
            }
        } else if (xbox.getLeftStickY() < -0.4) {
            if (!liftTrimTriggered) {
                robot.lift.trim(1.0);
                liftTrimTriggered = true;
            }
        } else {
            liftTrimTriggered = false;
        }
        /////////////////////////////////////////////////
        if((xbox.getLeftStickX() > 0.2) || (xbox.getLeftStickX() < -0.2)){
            

        }
        /////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////
        if (xbox.getRightStickY() > 0.4) {
            if (!wristTrimTriggered) {
                robot.wrist.trim(-0.5);
                wristTrimTriggered = true;
            }
        } else if (xbox.getRightStickY() < -0.4) {
            if (!wristTrimTriggered) {
                robot.wrist.trim(0.5);
                wristTrimTriggered = true;
            }
        } else {
            wristTrimTriggered = false;
        }
        ////////////////////////////////////////////

        if((xbox.getDPADX() > 0.2)){
            robot.climberUp();
        }
        if((xbox.getDPADX() < -0.2) && robot.climberUp){
            fsm.setGoalState(Team1323RobotFSM.LEVEL_ONE_PRE_HANG);
            
        }
        if(fsm.getCurrentState() == Team1323RobotFSM.LEVEL_ONE_PRE_HANG){
            if(robot.wrist.onTargetNow() && fsm.getCurrentState() != Team1323RobotFSM.LEVEL_ONE_READY_FOR_HANG){
                fsm.setGoalState(Team1323RobotFSM.LEVEL_ONE_READY_FOR_HANG);
            }
        }
        if(fsm.getCurrentState() == Team1323RobotFSM.LEVEL_ONE_READY_FOR_HANG){
            robot.pushrodUp();
            Timer.delay(1.0);
            robot.climberDown();
            fsm.setGoalState(Team1323RobotFSM.LEVEL_ONE_DONE);
        }
        ///////////////////////////////////////////////
        if(xbox.getLeftStick()){
            fsm.setGoalState(Team1323RobotFSM.DUMP);
        }              

        ///////////////////////////////////////////////
        if(xbox.getRightStick()) {
            if(!rightCenterClick){
                robot.shooter.switchMag();
                rightCenterClick = true;
            }
        }else{
            rightCenterClick = false;
        }
    }
    double volts = 0;
    public void driveUpdate() {

        if(right.getRawButton(4)){
//            robot.lift.lowerP();
            robot.turnToHeading(-Constants.ADJUST_TURN_DEG,1.5);
//           robot.nav.resetRobotPosition(0, 0, 0);
        }
        
        if(right.getRawButton(5)){
            robot.turnToHeading(Constants.ADJUST_TURN_DEG,1.5);
//            robot.lift.upP();
        }
        if(right.getRawButton(10)){
            robot.turn.downP();
        }
        
        if(right.getRawButton(11)){
            robot.turn.upP();
        }
        if(right.getRawButton(8)){ //left 7
          robot.turn.downD();
            /*
            if(SmartDashboard.getBoolean("found", false)){
                double newAngle = SmartDashboard.getNumber("azimuth", 0) - robot.nav.getRawHeading();
                robot.turnToHeading(newAngle,1.2);
            }
            if(robot.nav.topGoalFound()){
                robot.turnToHeading(robot.nav.topGoalAngle(), 0.5);
            }*/
        }
        if(right.getRawButton(6)){
            robot.turn.upD();
        }
        if (right.getRawButton(1)){robot.dt.lowGear();}
        if(right.getRawButton(3)){robot.dt.highGear(); }
        if(!robot.autoTurn){
            robot.dt.cheesyDrive(left.getX(), -right.getY(), left.getLeftBumper());
            if(robot.dt.gear){
                SmartDashboard.putString("ROBOT_GEAR", "HIGH");
            }else{
                SmartDashboard.putString("ROBOT_GEAR", "LOW");
            }
        }

           
        //////////
       
        if(right.getRawButton(9) && right.getRawButton(10) && right.getRawButton(11)){
            robot.climberTrigger.set(true);
        }

    }
    
    public void tunerUpdate(){
        
        if(right.getRawButton(6)){
            robot.shooterSpeed.upPIDp();
        }
        if(right.getRawButton(7)){
            robot.shooterSpeed.downPIDp();
        }
        if(right.getRawButton(11)){
            robot.shooterSpeed.upPIDd();
        }
        if(right.getRawButton(10)){
            robot.shooterSpeed.downPIDd();
        }
    }
}
