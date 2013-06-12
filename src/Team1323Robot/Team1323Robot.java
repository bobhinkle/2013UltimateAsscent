/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package Team1323Robot;


import IO.TeleController;
import Utilities.Constants;
import Utilities.Util;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Team1323Robot extends SimpleRobot {
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    TeleController control;
    Robot robot;
    Team1323RobotFSM fsm;
    private boolean autonCompleted = false;       
    private turnThread th;
    double matchTime;
    private Timer systemTime;
    public Team1323Robot() {
        control = TeleController.getInstance(); 
        robot = Robot.getInstance();
        fsm = Team1323RobotFSM.getInstance();
        fsm.start();
        this.getWatchdog().setEnabled(false);
        
        //Let each system load a couterpart subsystem
        robot.shooter.loadParts();
        robot.lift.loadParts();
        robot.wrist.loadParts();
        robot.turn.loadParts();
        robot.dist.loadParts();
        control.loadParts();
        robot.loadParts();
        systemTime = new Timer();
    }
    public void autonomous() {
        
        systemTime.start();
        
        int auton = 2;
        double shots = 0;
        autonCompleted = false;
        int numOfFrisbees = 2;
        double pyramidWidth = 94;
        double distToCenterline = 108.0; //108.5
        double driveOffset   = -1.0;
        double startingAngle = -13.5;
        double pickUpOffset = 24.0;
        double backUpOffset = -24.0;
        double D1 = distToCenterline; // Distance to center line from back leg
        double A  = 0; // Angle to outermost frisbee
        double H  = 0; // Hypotenous distance to travel to centerline and outermost frisbee
        double D4 = 0; // distance from outer leg and center line to first frisbee
        double A2 = 0; // total angle change from starting angle plus new angle
        double D5 = 44; // distance to travel to pick up 4 frisbees
        double D6 = 0; // distance to center of pyramid after picking up 4 frisbees
        double A3 = 0; // angle from A2 to parralel with centerline
        double error = 0;
        double current = 0;
        double A4 = 82; // Angle to center targer from back center line
        double pickUpPower = 0.83;
        double pickUpTimeout = 0.3;
        double backUpPower   = 1.0;        
        double backUpTimeout = 0.8;
        double pyramidToCenterlineTimeout = 10;
        double distanceDriven = 0;
        /*
        if(numOfFrisbees == 4){
            D5 = 90;
            D6 = -(D5 + backUpOffset);
            H  = -(distToCenterline + driveOffset);
            A2 = 0;
            A3 = -90;
            A4 = -8;
            /*
            A2 = -startingAngle;
            A3 = -(90);
        }else if(numOfFrisbees > 4){
            D4 = 11;
            H  = -Math.sqrt((D1 * D1) + (D4 * D4));
            A2 = -Util.aTan(D4,D1) + startingAngle;
            A3 = -(90 + A2);
        }else{
            D4 = 47 - ((4 - numOfFrisbees) * 11);
            D6 = D5 + D4 + backUpOffset + 20;
            A2 = 24.0 + startingAngle;
            H  = -103.5;
//            H  = -Math.sqrt((D1 * D1) + (D4 * D4));
//            A2 = Util.aTan(D4,D1)  + startingAngle;
            A3 = -(90 + A2);
        }*/
        D5 = 90;
        D6 = -(D5 + backUpOffset);
        H  = -(distToCenterline + driveOffset);
        A2 = 0;
        A3 = -90;
        A4 = -10;
        
        System.out.println("A:" + A + " H:" + H + " D4:" + D4 + " A2:" + A2 + " D5:" + D5 + " D6:" + D6 + " A3:" + A3);
        if(!autonCompleted && isAutonomous() && isEnabled()){
            switch(auton){
                
                
                case -1:
                    robot.nav.resetRobotPosition(0, 0, 0,true);
                    break;
                case 0:
                    robot.nav.resetRobotPosition(0, 0, 0,true);
                    
                    robot.shooterSpeed.fullCourtShot();
                    turnToHeading(startingAngle,2.5);
                    fsm.setGoalState(Team1323RobotFSM.SHOOT_PYRAMID_SIDESHOT);
                    shots = 6;
                    while((shots > 0) && isAutonomous()){
                        if(robot.shooterSpeed.fastonTarget2() && robot.wrist.onTargetNow()){
                            robot.shooter.shoot();
                            shots--;
                        }else if(robot.wrist.onTargetNow() && robot.shooterSpeed.encoderMalfunction()){
                            robot.shooter.shoot();
                            shots--;
                            Timer.delay(0.25);
                        }
                    }
                    robot.shooterSpeed.stop();
                    fsm.setGoalState(Team1323RobotFSM.FLOOR_STOW);
//                    driveDistanceHoldingHeading(H,0,1.2,2,2.0,false,0);
                    break;
                case 1:
                    robot.dt.setGear(Constants.HIGH_GEAR);
                    robot.nav.resetRobotPosition(0, 0, 0,true);
                    
                    robot.shooterSpeed.fullCourtShot();
                    fsm.setGoalState(Team1323RobotFSM.AUTON_FLOOR_NO_LIFT);
                    robot.shooterSpeed.fullCourtShot();
                    turnToHeading(startingAngle,0.75);
                    robot.shooterSpeed.fullCourtShot();
                    error = robot.turn.getError();
                    System.out.println("E1" + error);
                    shots = 3;
                    while((shots > 0) && isAutonomous()){
                        if(robot.shooterSpeed.fastonTarget2() && robot.wrist.onTargetNow()){
                            robot.shooter.shoot();
                            shots--;
                        }else if(robot.wrist.onTargetNow() && robot.shooterSpeed.encoderMalfunction()){
                            robot.shooter.shoot();
                            shots--;
                            Timer.delay(0.25);
                        }
//                        System.out.println(robot.shooter.getShooterSpeed() + " " + robot.wrist.onTarget() + " " + robot.wrist.getCurrent() + " " + robot.wrist.getError());
                    }
                    System.out.println("Time 1 " + " " + systemTime.get());
                    fsm.setGoalState(Team1323RobotFSM.FLOOR_INTAKE);
                    robot.shooterSpeed.stop();
                     
                    robot.dist.resetDistance();
                    driveDistanceHoldingHeading(H,0,backUpPower,pyramidToCenterlineTimeout,1.3,false,0);
                    System.out.println("Time 2 " + " " + systemTime.get());
                    System.out.println("H" + robot.nav.getDistance());
                    error = robot.nav.getRawHeading() - (A2 - error);
                    System.out.println("E2" + error);
                   
                    turnToHeading(A3,1.2);
                    System.out.println("Time 3 " + " " + systemTime.get());
                    error = robot.turn.getError();
                    System.out.println("E3" + error);
                    robot.intake.allForward();
                    System.out.println("D5" + robot.nav.getDistance());
                    System.out.println("Time 4 " + " " + systemTime.get());
                    double distanceToDrive = 94;
                    if(numOfFrisbees == 2){
                        System.out.println("Dist-1" + distanceDriven);
                        distanceDriven += driveDistanceHoldingHeading(distanceToDrive*(2/10.0),-90,1.0,1.0,1.0,true,0);
                        System.out.println("Dist-2" + distanceDriven);
                        distanceDriven += driveDistanceHoldingHeading(distanceToDrive*(5/10.0),-90,0.75,1.0,1.0,true,0);
                        System.out.println("Dist-3" + distanceDriven);
                        System.out.println("Time -1 " + " " + systemTime.get());
                        distanceDriven += driveDistanceHoldingHeading(distanceToDrive*(2/10.0),-90,0.9,2.0,2.0,false,9.5);
                        System.out.println("Dist" + distanceDriven);
                        System.out.println("Time 5 " + " " + systemTime.get());
                        driveDistanceHoldingHeading(-(distanceDriven),-120,1.0,1.0,3.0,true,0);
                    }else{
                        System.out.println("Dist-1" + distanceDriven);
                        distanceDriven += driveDistanceHoldingHeading(distanceToDrive*(2/10.0),-90,0.75,1.0,1.0,true,0);
                        System.out.println("Dist-2" + distanceDriven);
                        distanceDriven += driveDistanceHoldingHeading(distanceToDrive*(5/10.0),-90,0.7,1.0,1.0,true,0);
                        System.out.println("Dist-4" + distanceDriven);
                        distanceDriven += driveDistanceHoldingHeading(distanceToDrive*(3/10.0),-90,0.75,2.0,2.0,false,9.5);
                        System.out.println("Dist" + distanceDriven);
                        distanceDriven += driveDistanceHoldingHeading(distanceToDrive*(2/10.0),-90,0.75,2.0,2.0,false,9.5);
                        System.out.println("Dist" + distanceDriven);
                        System.out.println("Time 5 " + " " + systemTime.get());
                        driveDistanceHoldingHeading(-(distanceDriven),-120,1.0,1.0,3.0,true,0);
                    }
                    
                    fsm.setGoalState(Team1323RobotFSM.AUTON_BACK_NO_LIFT);
                    robot.shooterSpeed.fullCourtShot();
                    turnToHeading(A4,2.0);
                    robot.intake.allStop();
                    System.out.println("D6-1" + robot.nav.getDistance());
                    
                    error = robot.turn.getError();
                    System.out.println("E4" + error);
                    System.out.println("Time 6 " + " " + systemTime.get());
                    shots = 5;
                    while((shots >= 0) && isAutonomous()){
                        if(robot.shooterSpeed.fastonTarget2() && robot.wrist.onTargetNow()){
                            robot.shooter.shoot();
                            shots--;
                        }else if(robot.wrist.onTargetNow() && robot.shooterSpeed.encoderMalfunction()){
                            robot.shooter.shoot();
                            shots--;
                            Timer.delay(0.25);
                        }
 //                       System.out.println(robot.shooter.getShooterSpeed() + " " + robot.wrist.onTarget() + " " + robot.wrist.getCurrent() + " " + robot.wrist.getError());
                    }
                    System.out.println("Time 7 " + " " + systemTime.get());
                    fsm.setGoalState(Team1323RobotFSM.FLOOR_STOW);
                    robot.shooterSpeed.stop();
                    autonCompleted = true;
                    break;
                case 2:
                    robot.dt.setGear(Constants.HIGH_GEAR);
                    robot.nav.resetRobotPosition(0, 0, 0,true);
                    robot.shooterSpeed.fullCourtShot();
                    robot.intake.allForward();
                    Timer.delay(.2);
                    robot.intake.allStop();
                    fsm.setGoalState(Team1323RobotFSM.SHOOT_PYRAMID_UNDER);
                    driveDistanceHoldingHeading(8,0,1,1,2,false,0); // put to 0
                    System.out.println("Time 1 " + " " + systemTime.get());
                    shots = 3;
                    while((shots > 0) && isAutonomous()){
                        if(robot.shooterSpeed.fastonTarget2() && robot.wrist.onTargetNow()){
                            robot.shooter.shoot();
                            shots--;
                        }else if(robot.wrist.onTargetNow() && robot.shooterSpeed.encoderMalfunction()){
                            robot.shooter.shoot();
                            shots--;
                            Timer.delay(0.3);
                        }
//                        System.out.println("SP" + robot.shooter.getShooterSpeed() + " " + robot.wrist.onTarget() + " " + robot.wrist.getCurrent() + " " + robot.wrist.getError());
                    }
                    System.out.println("Time 2 " + " " + systemTime.get());
                    robot.shooterSpeed.stop();
                    fsm.setGoalState(Team1323RobotFSM.FLOOR_INTAKE);
                    while(robot.lift.getHeight() > 0 && robot.wrist.getAngle() > 2){
                        Timer.delay(.1);
                    }
                    Timer.delay(0.3);
                    robot.intake.allForward();
                    System.out.println("Time 3 " + " " + systemTime.get());
                    
                    double totalDistance = 182; //188
                    distanceDriven +=driveDistanceHoldingHeading(totalDistance*(6/20.0),0,0.85,0,1.0,true,0); //put to 0
                    System.out.println("D1-1 " + distanceDriven + " " + systemTime.get()); 
                    distanceDriven +=driveDistanceHoldingHeading(totalDistance*(2/20.0),-1,1.0,0,1.0,true,0);//put to 0
                    System.out.println("D1-2 " + distanceDriven + " " + systemTime.get()); 
                    distanceDriven +=driveDistanceHoldingHeading(totalDistance*(7/20.0),-6,0.86,1.0,1.0,true,9.0); 
                    distanceDriven +=driveDistanceHoldingHeading(totalDistance*(5/20.0),-10,0.97,3.0,1.0,false,9.0); //leave time 
                    
                    System.out.println("D1-3 " + distanceDriven + " " + systemTime.get()); 
                    totalDistance = totalDistance - distanceDriven - 100;
//                    totalDistance = -61;
                    System.out.println("DD-D" + totalDistance);
                    driveDistanceHoldingHeading(totalDistance*(1/4.0),0,1.0,1.0,1.0,true,0); //put to 0
                    robot.intake.allStop();
                    fsm.setGoalState(Team1323RobotFSM.AUTON_FRONT_LIFT);
                    robot.shooterSpeed.fullCourtShot();
//                    System.out.println("Time 5 " + " " + systemTime.get());
                    driveDistanceHoldingHeading(totalDistance*(3/4.0),1,1.0,1.0,2.0,false,0);//put to 0
                    System.out.println("Time 6 " + " " + systemTime.get());
                    Timer.delay(0.5);
                    shots = 5;
                    while((shots > 0) && isAutonomous()){
                        if(robot.shooterSpeed.fastonTarget2() && robot.wrist.onTargetNow()){
                            robot.shooter.slowShot();
                            shots--;
                        }else if(robot.wrist.onTargetNow() && robot.shooterSpeed.encoderMalfunction()){
                            robot.shooter.shoot();
                            shots--;
                            Timer.delay(0.25);
                        }
//                        System.out.println(robot.shooter.getShooterSpeed() + " " + robot.wrist.onTarget() + " " + robot.wrist.getCurrent() + " " + robot.wrist.getError());
                    }
                    System.out.println("Time 7 " + " " + systemTime.get());
                    fsm.setGoalState(Team1323RobotFSM.FLOOR_STOW);
                    robot.shooterSpeed.stop();
                    autonCompleted = true;
                    break;
            }
        }
        
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
 //       robot.comp.run();
        while(isEnabled() && isOperatorControl()){
            control.driveUpdate();
            control.coPilot();
            if(Constants.TUNING_MODE){
                control.tunerUpdate();
            }
            Timer.delay(0.1);
        }
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
        control.driveUpdate();
    }
    
    public double driveDistanceHoldingHeading(double distance, double heading,double maxSpeed,double timeout,double tol, boolean holdSpeed,double breakTime){                
        
        robot.dist.resetDistance();
        double startDistance = robot.nav.getDistance();
        double distanceChange = distance + startDistance;
        System.out.println("DD: " + startDistance + " " + distanceChange + " " + distance);
        robot.dist.reset();
        robot.dist.setGoal(distanceChange, maxSpeed,heading,timeout,tol,holdSpeed);
        if(breakTime > 0){
            while(!robot.dist.onTarget() && isAutonomous() && (systemTime.get() < breakTime)){
                robot.dist.run();
            }
        }else{
            while(!robot.dist.onTarget() && isAutonomous()){
                robot.dist.run();
            }
        }
        robot.dt.directDrive(0, 0);
        return robot.nav.getDistance() - startDistance;
    }
    public void turnToHeading(double heading,double timeout){
        robot.turn.reset();
        robot.turn.setGoal(heading,timeout);
        th = new turnThread();
        th.run();
    }
    private class turnThread extends Thread{
        
        public void run(){
            try {
                while(!robot.turn.onTarget() && !control.right.getRawButton(2) && (Util.inRange(control.left.getX(), 0.2))){
                    robot.turn.run();
                }
                
                robot.dt.directDrive(0, 0);
            }catch(Exception e){
                
            }
        }
    }
}
