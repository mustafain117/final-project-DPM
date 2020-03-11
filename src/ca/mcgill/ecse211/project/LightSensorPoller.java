package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.BLUE_MEAN;
import static ca.mcgill.ecse211.project.Resources.BLUE_SD;
import static ca.mcgill.ecse211.project.Resources.GREEN_MEAN;
import static ca.mcgill.ecse211.project.Resources.GREEN_SD;
import static ca.mcgill.ecse211.project.Resources.ORANGE_MEAN;
import static ca.mcgill.ecse211.project.Resources.ORANGE_SD;
import static ca.mcgill.ecse211.project.Resources.YELLOW_MEAN;
import static ca.mcgill.ecse211.project.Resources.YELLOW_SD;
import  static ca.mcgill.ecse211.project.Resources.colorSensorLeft;
import static ca.mcgill.ecse211.project.Resources.colorSensorRight;
import static ca.mcgill.ecse211.project.Resources.lightSensor;

/**
  * Runs as a thread to collect samples from all three light sensors and characterises colour detected by middle color sensor 
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
  
  /**
   * Possible color types detected by color sensor
   */
  public enum COLOUR {
    BLUE, GREEN, YELLOW, ORANGE, NONE;
  }
  
  /**
   * Color detected by the middle color sensor
   */
  private  COLOUR detectedColour;
  
  /**
   * Records samples from all three light sensors into colourData, colorRight and colorLeft arrays. Calls {@code updateDetectedColour} to continuously update the color detected by middle color sensor.
   */
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
      
      float colourRed = colourData[0];
      float colourGreen = colourData[1];
      float colourBlue = colourData[2];
      
      float normRed = (float) ((colourRed)/Math.sqrt(colourRed*colourRed+colourGreen*colourGreen+colourBlue*colourBlue));
      float normGreen = (float) ((colourGreen)/Math.sqrt(colourRed*colourRed+colourGreen*colourGreen+colourBlue*colourBlue));
      float normBlue = (float) ((colourBlue)/Math.sqrt(colourRed*colourRed+colourGreen*colourGreen+colourBlue*colourBlue));
      
      updateDetectedColour(normRed, normGreen, normBlue);
     
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
  
  /**
   * Characterises the color detected by the color sensor and the {@code detectedColor} variable which can be accessed 
   * using the {@code getColor} method.
   * @param colourRed Normalized red value from color sensor
   * @param colourGreen Normalized green value from color sensor
   * @param colourBlue Normalized blue value from color sensor
   */
  public  void updateDetectedColour(double colourRed, double colourGreen, double colourBlue) {
    this.detectedColour  = COLOUR.NONE;
    
    int stdDevs = 40;
    //for orange and yellow;
    int stDevsOY = 30;
    
    if ((colourBlue >= GREEN_MEAN[2] - stdDevs* GREEN_SD[2]) & (colourBlue <= GREEN_MEAN[2] + stdDevs*GREEN_SD[2])) {

      if ((colourRed >= GREEN_MEAN[0] - stdDevs*GREEN_SD[0]) & (colourRed <= GREEN_MEAN[0] + stdDevs*GREEN_SD[0])) {

        if ((colourGreen >= GREEN_MEAN[1] - stdDevs*GREEN_SD[1]) & (colourGreen <= GREEN_MEAN[1] +stdDevs* GREEN_SD[1])) {

          this.detectedColour = COLOUR.GREEN;
        }
      }
    } 

    if ((colourBlue >= BLUE_MEAN[2] - stdDevs*BLUE_SD[2]) && (colourBlue <= BLUE_MEAN[2] +stdDevs* BLUE_SD[2])) {

      if ((colourRed >= BLUE_MEAN[0] -stdDevs* BLUE_SD[0]) && (colourRed <= BLUE_MEAN[0] +stdDevs* BLUE_SD[0])) {

        if ((colourGreen >= BLUE_MEAN[1] - stdDevs*BLUE_SD[1]) && (colourGreen <= BLUE_MEAN[1] +stdDevs* BLUE_SD[1])) {

          this.detectedColour = COLOUR.BLUE;
        }
      }
    } 
    

    if ((colourBlue >= YELLOW_MEAN[2] - stDevsOY*YELLOW_SD[2]) && (colourBlue <= YELLOW_MEAN[2] + stDevsOY*YELLOW_SD[2])) {

      if ((colourRed >= YELLOW_MEAN[0] - stDevsOY*YELLOW_SD[0]) && (colourRed <= YELLOW_MEAN[0] + stDevsOY*YELLOW_SD[0])) {

        if ((colourGreen >= YELLOW_MEAN[1] - stDevsOY*YELLOW_SD[1]) && (colourGreen <= YELLOW_MEAN[1] +stDevsOY* YELLOW_SD[1])) {

          this.detectedColour = COLOUR.YELLOW;
        }
      }

    }  
    if ((colourBlue >= ORANGE_MEAN[2] - stdDevs*ORANGE_SD[2]) && (colourBlue <= ORANGE_MEAN[2] + stdDevs*ORANGE_SD[2])) {

      if ((colourRed >= ORANGE_MEAN[0] - stdDevs*ORANGE_SD[0]) && (colourRed <= ORANGE_MEAN[0] + stdDevs*ORANGE_SD[0])) {

        if ((colourGreen >= ORANGE_MEAN[1] - stdDevs*ORANGE_SD[1]) && (colourGreen <= ORANGE_MEAN[1] + stdDevs*ORANGE_SD[1])) {

          this.detectedColour = COLOUR.ORANGE;
        }
      }
    }
  }
/**
 * getter for color detected by color sensor
 * @return the COLOUR type detected (GREEN, BLUE, ORANGE, YELLOW, NONE)
 */
  public COLOUR getColor() {
    return this.detectedColour;
  }
}
