package ca.mcgill.ecse211.project;

import  static ca.mcgill.ecse211.project.Resources.colorSensorLeft;
import static ca.mcgill.ecse211.project.Resources.colorSensorRight;
import static ca.mcgill.ecse211.project.Resources.lightSensor;
 /**
  * Runs as a thread to collect samples from all three light sensors
  * @author Mustafain 
  *
  */
public class LightSensorPoller implements Runnable {
  private static final long PERIOD = 1000;

  /*
   * Float array to store rgb sample from middle color sensor
   */
  private float[] colourData = new float[3];
  /*
   * Float array to store sample from right color sensor
   */
  private static float colorRight[] = new float[colorSensorRight.sampleSize()];
  /*
   * Float array to store sample from left color sensor
   */
  private static float colorLeft[]= new float[colorSensorLeft.sampleSize()];
  
  public void run() {
    long updateStart;
    long updateDuration;
    updateStart = System.currentTimeMillis();
    
    while(true) {
      
      //colllecting samples from left and right sensor
      colorSensorLeft.getRedMode().fetchSample(colorLeft, 0);
      colorSensorRight.getRedMode().fetchSample(colorRight, 0);
     
      //rgb sample
      lightSensor.getRGBMode().fetchSample(colourData, 0);
     
      updateDuration = System.currentTimeMillis() - updateStart;
      if (updateDuration < PERIOD) {
        try {
          Thread.sleep(PERIOD - updateDuration);
        }catch(InterruptedException e) {
          
        }
      }
    }
  }
 /**
  * getter for scaled sample from right color sensor
  * @return scaled sample from right color sensor
  */
  public static int getRightColorVal() {
    return (int) (colorRight[0] * 100);
  }

  /**
   * getter for scaled sample from left color sensor
   * @return scaled sample from left color sensor
   */
  public static int getLeftColorVal() {
    return (int) (colorLeft[0] * 100);
  }
}
