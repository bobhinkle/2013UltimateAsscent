package Sensors;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 * @author frc2620
 */
public class CameraDistanceCalc {
    
    private AxisCamera camera_; 
    private CriteriaCollection cc;
    private DriverStationLCD b_LCD;
    
    public CameraDistanceCalc(AxisCamera cam)
    {
        camera_ = cam;
        cc = new CriteriaCollection();      
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false);
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false);
        b_LCD = DriverStationLCD.getInstance();
        b_LCD.updateLCD();
    }
    
    public CameraDistanceCalc(String cameraIP)
    {
        camera_ = AxisCamera.getInstance(cameraIP);
        cc = new CriteriaCollection();      
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false);
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false);
        b_LCD = DriverStationLCD.getInstance();
        b_LCD.updateLCD();
    }
    
    public double calcDistance()
    {
        double distance = 0;
        try 
        {
            ColorImage image = camera_.getImage();   
            //ColorImage image;                           
            //image =  new RGBImage("/10ft2.jpg");
            //BinaryImage thresholdImage = image.thresholdRGB(25, 255, 0, 45, 0, 47);   // keep only red objects
            BinaryImage thresholdImage = image.thresholdRGB(0, 45, 25, 225, 0, 45);     // Keep only green objects
            BinaryImage bigObjectsImage = thresholdImage.removeSmallObjects(false, 2);  // remove small artifacts
            BinaryImage convexHullImage = bigObjectsImage.convexHull(false);            // fill in occluded rectangles
            BinaryImage filteredImage = convexHullImage.particleFilter(cc);             // find filled in rectangles

            ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();  // get list of results
      
            ParticleAnalysisReport ThreePtn = reports[reports.length-1]; // Reports is listed by largest to smallest area in the rectangle, 3 ptner should be the smallest area
            
            double tw = 4.5;
            double tpw = ThreePtn.boundingRectWidth;
            double cw = 320;
            
            double WFOV = tw * (cw / tpw);
            
            distance = (WFOV / 2) / (MathUtils.atan(27 * (Math.PI / 180)));
            
            
            System.out.println(filteredImage.getNumberParticles() + "  " + Timer.getFPGATimestamp());

            SmartDashboard.putNumber("Number of Particles: ", filteredImage.getNumberParticles());
            SmartDashboard.putNumber("Height:  ", filteredImage.getHeight());
            SmartDashboard.putNumber("Width:   ", filteredImage.getWidth());
            SmartDashboard.putNumber("Distance:", distance);

            filteredImage.free();
            convexHullImage.free();
            bigObjectsImage.free();
            thresholdImage.free();
            image.free();

        } catch (AxisCameraException ex) {
            ex.printStackTrace();
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        }
        return distance;
    }
}