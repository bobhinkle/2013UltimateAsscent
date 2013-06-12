package Utilities;

import com.sun.squawk.util.MathUtils;

public class Util {

    /**
     * carful with negative
     * 
     * @param val 
     * @param min
     * @param max
     * @return 
     */
    public static double speedLimit(double speed, double position, double min, double max) {
        if((position < min && speed < 0) || (position > max && speed > 0))
            return 0.0;
        else
            return speed;
    } 
    public static double getDifferenceInAngleDegrees(double from, double to)
    {
        return boundAngleNeg180to180Degrees(to-from);
    }
    public static double boundAngleNeg180to180Degrees(double angle)
    {
        // Naive algorithm
        while(angle >= 180.0)
        {
            angle -= 360.0;
        }
        while(angle < -180.0)
        {
            angle += 360.0;
        }
        return angle;
    }
    public static double pidPower(double power,double minReverse, double maxReverse, double minForward, double maxForward){
        if(maxReverse > minReverse){
            maxReverse = minReverse;
        }
        if(maxForward < minForward){
            maxForward = minForward;
        }
        if(power < 0){
            if(power > minReverse){
                return minReverse;
            }else if(power < maxReverse){
                return maxReverse;
            }else{
                return power;
            }
        }else{
            if(power < minForward){
                return minForward;
            }else if(power > maxForward){
                return maxForward;
            }else{
                return power;
            }
        }
    }

    /**
     * 
     * @param val
     * @param min
     * @param max
     * @return 
     */
    public static double limit(double val, double min, double max) {
        if (min > max) return 0.0;
        if (val > max) return max;
        if (val < min) return min;
        return val;
    } 
    
    /**
     * 
     * @param val
     * @param abs
     * @return 
     */
    public static double limit(double val, double abs){
        if (val > abs) 
            return abs;
        else if (val < -abs) 
            return -abs;
        else
            return val;
    }  
    
    public static double limit(double val){
        if (val > 1) 
            return 1;
        else if (val < -1) 
            return -1;
        else
            return val;
    }  
    
    public static double buffer(double goalValue, double storedValue, int strength) { 
        return ((storedValue * strength) + (goalValue * (100 - strength))) / 100;
    }

    public static double deadBand(double val, double deadband){
        if (val < deadband && val > -deadband) 
            return 0.0;
        else 
            return val;
    }
    public static double deadBandBump(double val, double deadband){
        if (val < deadband && val > 0){ 
            return deadband;
        }else if(val > -deadband && val < 0){
            return -deadband;
        }else{
            return val;
        }
    }
    public static boolean onTarget(double target, double current, double error){
        return ((Math.abs(current) < (Math.abs(target)+ Math.abs(error))) && (Math.abs(current) > (Math.abs(target)- Math.abs(error))));
    }
    public static boolean inRange(double val, double maxAbsError) {
        return (Math.abs(val) < maxAbsError);
    }

    public static boolean inRange(double val, double minError, double maxError) {
        return (val > minError && val < maxError);
    }
    
    public static double aTan(double opp, double adj) {
        return Math.toDegrees(MathUtils.atan2(opp, adj)); 
    }
    
    public static double aSin(double opp, double hyp) {
        return Math.toDegrees(MathUtils.asin(opp / hyp)); 
    }

    public static double victorLinearize(double goal_speed) { //Kevin wants to eliminate the need for this
        final double deadband_value = 0.082;
        if (goal_speed > deadband_value)
            goal_speed -= deadband_value;
        else if (goal_speed < -deadband_value)
            goal_speed += deadband_value;
        else
            goal_speed = 0.0;
        goal_speed = goal_speed / (1.0 - deadband_value);

        double goal_speed2 = goal_speed * goal_speed;
        double goal_speed3 = goal_speed2 * goal_speed;
        double goal_speed4 = goal_speed3 * goal_speed;
        double goal_speed5 = goal_speed4 * goal_speed;
        double goal_speed6 = goal_speed5 * goal_speed;
        double goal_speed7 = goal_speed6 * goal_speed;

        // Original untweaked one.
        //double victor_fit_c		= -1.6429;
        //double victor_fit_d		= 4.58861e-08;
        //double victor_fit_e		= 0.547087;
        //double victor_fit_f		= -1.19447e-08;

        // Constants for the 5th order polynomial
        double victor_fit_e1		= 0.437239;
        double victor_fit_c1		= -1.56847;
        double victor_fit_a1		= (- (125.0 * victor_fit_e1  + 125.0 * victor_fit_c1 - 116.0) / 125.0);
        double answer_5th_order = (victor_fit_a1 * goal_speed5
                        + victor_fit_c1 * goal_speed3
                        + victor_fit_e1 * goal_speed);

        // Constants for the 7th order polynomial
        double victor_fit_c2 = -5.46889;
        double victor_fit_e2 = 2.24214;
        double victor_fit_g2 = -0.042375;
        double victor_fit_a2 = (- (125.0 * (victor_fit_c2 + victor_fit_e2 + victor_fit_g2) - 116.0) / 125.0);
        double answer_7th_order = (victor_fit_a2 * goal_speed7
                        + victor_fit_c2 * goal_speed5
                        + victor_fit_e2 * goal_speed3
                        + victor_fit_g2 * goal_speed);


        // Average the 5th and 7th order polynomials
        double answer =  0.85 * 0.5 * (answer_7th_order + answer_5th_order)
                + .15 * goal_speed * (1.0 - deadband_value);

        if (answer > 0.001)
            answer += deadband_value;
        else if (answer < -0.001)
            answer -= deadband_value;

        return answer;
    }
    
    public static double boundAngle0to360Degrees(double angle)
    {
        // Naive algorithm
        while(angle >= 360.0)
        {
            angle -= 360.0;
        }
        while(angle < 0.0)
        {
            angle += 360.0;
        }
        return angle;
    }
    
    public static double scale(double x, double from_min, double from_max, double to_min, double to_max)
    {
        if(x < from_min)
            x = from_min;
        else if(x > from_max)
            x = from_max;
        return ((x-from_min)*(to_max-to_min)/(from_max-from_min)) + to_min;
    }
}
