/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Team1323Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import Utilities.Constants;

/**
 *
 * @author Eugene Fang
 */
public class Team1323RobotFSM extends Thread {
    private Robot robot;
    private static Team1323RobotFSM instance = null;
    
    // Steady States
    public final static int INIT = 0;
    public final static int FLOOR_INTAKE = 100;
    public final static int FLOOR_STOW = 101;
    public final static int SHOOT_FULL_COURT = 102;
    public final static int SHOOT_PYRAMID_SIDESHOT = 103;
    public final static int SHOOT_PYRAMID_CENTERLINE = 104;
    public final static int SHOOT_PYRAMID_UNDER = 105;
    public final static int MAG_TO_LOWER = 106;
    public final static int HUMAN_LOAD = 107;
    public final static int AUTON_CENTERLINE_SHOT = 108; 
    public final static int DUMP = 109;
    public final static int AUTON_FLOOR_NO_LIFT = 110;
    public final static int AUTON_BACK_NO_LIFT = 111;
    public final static int AUTON_FRONT_LIFT = 112;
    public final static int LEVEL_ONE_PRE_HANG = 113;
    public final static int LEVEL_ONE_READY_FOR_HANG = 114;
    public final static int LEVEL_ONE_DONE = 115;
    
    private boolean keepRunning = true;
    private int currentState = INIT;
    private int goalState = -1;
    public static Team1323RobotFSM getInstance()
    {
        if( instance == null )
            instance = new Team1323RobotFSM();
        return instance;
    }
        
    public Team1323RobotFSM() {
        robot = Robot.getInstance();
    }

    public void kill() {
        keepRunning = false;
    }
    public void setGoalState(int goal) {
        goalState = goal;
    }
    public int getGoalState(){
        return goalState;
    }
    private boolean checkStateChange(){
        if(goalState != currentState){
            return true;
       }
        return false;
    }
    public int getCurrentState() {
        return currentState;
    }
    private void stateComplete(int state){
        currentState = state;
    }
    public void run(){
        try {
            keepRunning = true;
            while(keepRunning){
                robot.lights.checkIfDetected();
                robot.wrist.run();
                robot.lift.run();
                robot.shooterSpeed.run();
                robot.nav.run();
                robot.shooter.updateEnc();
                SmartDashboard.putNumber("AIR_PRESSURE", robot.airPressure.getPressure());
                if(checkStateChange()){
                    switch(goalState){
                        case INIT:
                            SmartDashboard.putString("FSM_STATE", "INIT");
                            currentState = -1;
                            break;
                        case FLOOR_INTAKE:
                            SmartDashboard.putString("FSM_STATE", "FLOOR_INTAKE");
                            robot.shooterSpeed.stop();
                            robot.shooter.downMag(false);
                            robot.shooter.triggerIn();
                            robot.wrist.setGoal(Constants.WRIST_LOADING_ANGLE);
                            robot.lift.setGoal(Constants.ELEVATOR_STOW);
                            goalState = MAG_TO_LOWER;
                            break;
                        case MAG_TO_LOWER:
                            if(robot.shooter.getMagInStowPosition()){
                                SmartDashboard.putString("FSM_STATE", "MAG_TO_LOWER_1");
                                robot.shooter.downMag(false);
                            }else{
                                SmartDashboard.putString("FSM_STATE", "MAG_TO_LOWER");
                                stateComplete(FLOOR_INTAKE);
                                goalState = FLOOR_INTAKE;
                            }
                            break;
                        case DUMP:  // Unused
                            SmartDashboard.putString("FSM_STATE", "SHOOT_FULL_COURT");
                            robot.shooter.upMag();
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
                            robot.wrist.setGoal(Constants.WRIST_DUMP_ANGLE);
                            stateComplete(DUMP);
                            break;
                        case SHOOT_FULL_COURT:  // Unused
                            SmartDashboard.putString("FSM_STATE", "SHOOT_FULL_COURT");
                            robot.shooter.upMag();
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
                            robot.wrist.setGoal(Constants.WRIST_FULL_FIELD_ANGLE);
                            robot.wrist.setPlunger();
                            stateComplete(SHOOT_FULL_COURT);
                            break;
                        case SHOOT_PYRAMID_SIDESHOT:
                            SmartDashboard.putString("FSM_STATE", "SHOOT_PYRAMID_BOTTOM");
                            robot.shooter.upMag();
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
                            robot.wrist.setGoal(Constants.WRIST_SIDE_SHOT_ANGLE);
                            robot.shooterSpeed.fullCourtShot();
                            robot.wrist.setPlunger();
                            robot.humanLoader.set(0);
                            robot.intake.allStop();
                            stateComplete(SHOOT_PYRAMID_SIDESHOT);
                            break;
                        case SHOOT_PYRAMID_UNDER:
                            SmartDashboard.putString("FSM_STATE", "SHOOT_PYRAMID_UNDER");
                            robot.shooter.upMag();
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
//                            robot.lift.setGoal(Constants.ELEVATOR_MIN_HEIGHT);
                            robot.wrist.setGoal(Constants.WRIST_UNDER_PYRAMID_SHOT_ANGLE);
                            robot.shooterSpeed.fullCourtShot();
                            robot.wrist.setPlunger();
                            robot.humanLoader.set(0);
                            robot.intake.allStop();
                            stateComplete(SHOOT_PYRAMID_UNDER);
                            break;
                        case SHOOT_PYRAMID_CENTERLINE:
                            SmartDashboard.putString("FSM_STATE", "SHOOT_PYRAMID_TOP");
                            robot.shooter.upMag();
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
                            robot.wrist.setGoal(Constants.WRIST_FLOOR_SHOT_ANGLE);
                            stateComplete(SHOOT_PYRAMID_CENTERLINE);
                            robot.wrist.setPlunger();
                            break;
                        case AUTON_CENTERLINE_SHOT:
                            SmartDashboard.putString("FSM_STATE", "SHOOT_PYRAMID_TOP");
                            robot.shooter.upMag();
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
                            robot.wrist.setGoal(Constants.WRIST_AUTON_UNDER_PYRAMID_ANGLE);
                            stateComplete(AUTON_CENTERLINE_SHOT);
                            robot.wrist.setPlunger();
                            break;
                        case AUTON_FLOOR_NO_LIFT:
                            SmartDashboard.putString("FSM_STATE", "SHOOT_PYRAMID_TOP");
                            robot.shooter.upMag();
//                            robot.lift.setGoal(Constants.ELEVATOR_MIN_HEIGHT);
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
                            robot.wrist.setGoal(Constants.WRIST_SIDE_SHOT_ANGLE);
                            stateComplete(AUTON_FLOOR_NO_LIFT);
                            robot.wrist.setPlunger();
                            break;
                        case AUTON_BACK_NO_LIFT:
                            SmartDashboard.putString("FSM_STATE", "SHOOT_PYRAMID_TOP");
                            robot.shooter.upMag();
//                            robot.lift.setGoal(Constants.ELEVATOR_MIN_HEIGHT);
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
                            robot.wrist.setGoal(Constants.WRIST_AUTON_BACK_NO_LIFT_ANGLE);
                            stateComplete(AUTON_BACK_NO_LIFT);
                            robot.wrist.setPlunger();
                            break;
                        case FLOOR_STOW:
                            SmartDashboard.putString("FSM_STATE", "FLOOR_STOW");
                            robot.shooterSpeed.stop();
                            robot.shooter.upMag();
                            robot.shooter.triggerIn();
                            robot.lift.setGoal(Constants.ELEVATOR_MIN_HEIGHT);
                            robot.wrist.setGoal(Constants.WRIST_MIN_ANGLE);
                            stateComplete(FLOOR_STOW);
                            break;
                        case HUMAN_LOAD:
                            robot.shooterSpeed.stop();
                            robot.shooter.upMag();
                            robot.shooter.triggerIn();
                            robot.lift.setGoal(Constants.ELEVATOR_HUMAN_LOAD);
                            robot.wrist.setGoal(Constants.WRIST_HUMAN_LOAD_ANGLE);
                            SmartDashboard.putString("FSM_STATE", "HUMAN_LOAD");
                            stateComplete(HUMAN_LOAD);
                            break;
                        case AUTON_FRONT_LIFT:
                            SmartDashboard.putString("FSM_STATE", "SHOOT_PYRAMID_FRONT");
                            robot.shooter.upMag();
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
                            robot.wrist.setGoal(Constants.WRIST_AUTON_FRONT_OF_PYRAMID);
                            stateComplete(AUTON_FRONT_LIFT);
                            robot.wrist.setPlunger();
                            break;
                        case LEVEL_ONE_PRE_HANG:
                            SmartDashboard.putString("FSM_STATE", "LEVEL_ONE_PRE_HANG");
                            robot.shooterSpeed.stop();
                            robot.shooter.upMag();
                            robot.shooter.triggerIn();
                            robot.lift.setGoal(Constants.ELEVATOR_FLOOR_SHOT);
                            robot.wrist.setGoal(Constants.WRIST_CLIMBER_ANGLE);
                            stateComplete(LEVEL_ONE_PRE_HANG);
                            
                            break;
                        case LEVEL_ONE_READY_FOR_HANG:
                            SmartDashboard.putString("FSM_STATE", "LEVEL_ONE_READY");
                            robot.shooterSpeed.stop();
                            robot.shooter.upMag();
                            robot.shooter.triggerIn();
                            stateComplete(LEVEL_ONE_READY_FOR_HANG);
                            
                            break;
                        case LEVEL_ONE_DONE:
                            SmartDashboard.putString("FSM_STATE", "LEVEL_ONE_DONE");
                            
                            stateComplete(LEVEL_ONE_DONE);
                            
                            break;
                        }
                }
                Timer.delay(0.01);
            }

        }catch(Exception e){

        }
    }
    
}
