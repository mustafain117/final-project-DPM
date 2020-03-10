package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.DEG_PER_RAD;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.rightMotor;

/**
 * Class for odometry correction and robot localization using light sensors 
 */
public class LightLocalization {

  //Distance between sensors and front axel
  private static int fow=150;
  //Pull Back on the wheel to clear the line to ensure consistent trials
  private static int pullBack=-50;
  
  //Testing parameter for shifting the final orientation
  private static int lineThicknessCorrection=0;
 
  /**
   * Creates a LightLocalization object
   */
  public LightLocalization() {
  }
  
  /**
   * Light sensor correction along a single line used before and after the tunnel 
   * to insure straight line travel through said tunnel
   * 
   * @param turnBy initial angle to turn by to align axle along the line that we want to correct with
   */
  public void singleLineCorrection(double turnBy) {
    NavigatorUtility.turnBy(turnBy);
    NavigatorUtility.moveDistFwd(250, 100);
    
    while(true) {
      
      if(LightSensorPoller.getRightColorVal() <=30 && LightSensorPoller.getLeftColorVal()<=30) {
        rightMotor.stop(true);
        leftMotor.stop(true);
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      }
      
      
       if(LightSensorPoller.getRightColorVal() <=30) {

        rightMotor.stop(true);
        leftMotor.rotate(-pullBack);
        leftMotor.backward();
    
        
        while (LightSensorPoller.getLeftColorVal() >30) {}
        leftMotor.stop();
        leftMotor.rotate(-lineThicknessCorrection);
       
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      }
      
      
      else if(LightSensorPoller.getLeftColorVal() <=30  ) {

        leftMotor.stop(true);
        rightMotor.rotate(-pullBack);
        rightMotor.backward();
        
        while (LightSensorPoller.getRightColorVal() >30) {}
        rightMotor.stop(true);
        rightMotor.rotate(-lineThicknessCorrection);
  
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      }
     
      //if no line is detected keep moving backwards
      else{
        leftMotor.setSpeed(100); 
        rightMotor.setSpeed(100);
        leftMotor.backward();
        rightMotor.backward();
      }
    }
  }
  
  /**
   * Corrects the ev3 along the y axis, first part of the light correction
   */
  public  void odoCorrectionFirst(){
    while(true) {
      
      if(LightSensorPoller.getRightColorVal() <=30 && LightSensorPoller.getLeftColorVal()<=30) {
        rightMotor.stop(true);
        leftMotor.stop(true);
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        NavigatorUtility.turnBy(90);
        break;
      }
      
      
       if(LightSensorPoller.getRightColorVal() <=30) {

        rightMotor.stop(true);
        leftMotor.rotate(-pullBack);
        leftMotor.backward();
    
        
        while (LightSensorPoller.getLeftColorVal() >30) {}
        leftMotor.stop();
        leftMotor.rotate(-lineThicknessCorrection);
       
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        NavigatorUtility.turnBy(90);
        break;
      }
      
      
      else if(LightSensorPoller.getLeftColorVal() <=30  ) {

        leftMotor.stop(true);
        rightMotor.rotate(-pullBack);
        rightMotor.backward();
        
        while (LightSensorPoller.getRightColorVal() >30) {}
        rightMotor.stop(true);
        rightMotor.rotate(-lineThicknessCorrection);
  
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        NavigatorUtility.turnBy(90);
        break;
      }
     
      //if no line is detected keep moving backwards
      else{
        leftMotor.setSpeed(100); 
        rightMotor.setSpeed(100);
        leftMotor.backward();
        rightMotor.backward();
      }
    }
  }
  /**
   * Corrects the ev3 along the x axis, second part of light sensor correction
   */
  public void odoCorrectionSecond() {   
    //Go over line to then mov backwards into it
    NavigatorUtility.moveDistFwd(350,100);
    
    while(true) {
      
      if(LightSensorPoller.getRightColorVal() <=30 && LightSensorPoller.getLeftColorVal()<=30) {
        rightMotor.stop(true);
        leftMotor.stop(true);
        //leftMotor.rotate(pullBack);
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      }
      
      
       if(LightSensorPoller.getRightColorVal() <=30) {

        rightMotor.stop(true);
        //leftMotor.stop(true);
        leftMotor.rotate(pullBack);
        leftMotor.forward();
    
        
        while (LightSensorPoller.getLeftColorVal() >30) {}
        leftMotor.stop();
        leftMotor.rotate(lineThicknessCorrection);
       
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      }
      
      
      else if(LightSensorPoller.getLeftColorVal() <=30  ) {

        //rightMotor.stop(true);
        leftMotor.stop(true);
        rightMotor.rotate(pullBack);
        rightMotor.forward();
        
        while (LightSensorPoller.getRightColorVal() >30) {}
        rightMotor.stop(true);
        rightMotor.rotate(lineThicknessCorrection);
  
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      }
     
      //if no line is detected keep moving backwards
      else {
        leftMotor.setSpeed(100); 
        rightMotor.setSpeed(100);
        leftMotor.backward();
        rightMotor.backward();
        
      }
    }
  }
 
  /**
   * This method updates the odometer coordinates to that of the point the odoCorrection just localized to
   */
  public void updateOdometer() {
    //Correct coordinates
    odometer.setX(closestTile(odometer.getXyt()[0]));
    odometer.setY(closestTile(odometer.getXyt()[1]));
    
    double currentOdoTheta=odometer.getXyt()[2];
    
    //Correct theta
    if(currentOdoTheta<100/DEG_PER_RAD && currentOdoTheta>80/DEG_PER_RAD) {
    odometer.setTheta(90/Resources.DEG_PER_RAD);
    }
    else {
      odometer.setTheta(270/Resources.DEG_PER_RAD);

    }
  }
  
  /**
   * Method to find closest Tile
   * @param odo thought location by robot
   * @return closestTile
   */
  private double closestTile(double odo) {
    int closest= (int)( (odo/TILE_SIZE)+0.5) ;
    return closest*TILE_SIZE;
  }
  
  /**
   * Sleep the program for a given duration of time
   * @param duration The sleep duaration in milliseconds
   */
  private void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }
}