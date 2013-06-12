/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package Utilities;

import Team1323Robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author xpsl05x
 */
public class Output extends Thread{
    Robot robot;
    private boolean keepRunning = true;
    public Output(){
        robot = Robot.getInstance();
    }
    public void printOutput(){/*
        SmartDashboard.putNumber("SHOOTER_P", robot.shooter.pdc.getP());
        SmartDashboard.putNumber("SHOOTER_D", robot.shooter.pdc.getD());
        SmartDashboard.putNumber("SHOOTER_SPEED", robot.shooter.shooterSpeed());
        SmartDashboard.putNumber("SHOOTER_GOAL", robot.shooter.getGoal());
        
        SmartDashboard.putNumber("ELE_P", robot.lift.pidc.getP());
        SmartDashboard.putNumber("ELE_I", robot.lift.pidc.getI());
        SmartDashboard.putNumber("ELE_D", robot.lift.pidc.getD());
        SmartDashboard.putNumber("ELE_GOAL", robot.lift.getGoal());
        SmartDashboard.putNumber("ELE_HEIGHT", robot.lift.getHeight());
        */
        SmartDashboard.putNumber("WRIST_ANGLE", robot.wrist.getAngle());
    }
    
    public void run(){
         try {
                while(keepRunning){
                    printOutput();
                }
         Thread.sleep(250);
            } catch (Exception ex) {
                System.out.println("Index Thread Error");
         }
    }
}
