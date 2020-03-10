package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Main class to launch the program and acts as controller
 * @author Mustafain , Bruno
 */
public class Main {
  private static UltrasonicLocalization usLocalizer;
  private static LightLocalization lightLocalizer;

  /**
   * Main method starts up the odometer and LightSensorPoller threads, creates an instance of UltrasonicLocalization 
   * to localize and starts navigation and search process.
   * @param args Array of command line params
   */
  public static void main(String[] args) {
    //wait for user button left press
    Display.test();
    int buttonChoice;
    buttonChoice = Button.waitForAnyPress();
    
    if(buttonChoice != Button.ID_ESCAPE) {
      // start the odometer
      new Thread(odometer).start();
      
      usLocalizer = new UltrasonicLocalization();
      usLocalizer.doLocalization();
      
      new Thread(new LightSensorPoller()).start();
      //Traverse the first line (x=1) to use the odo correction method 
      lightLocalizer = new LightLocalization();
      Navigation navigation = new Navigation(lightLocalizer);
     
      NavigatorUtility.moveDistFwd((int) TILE_SIZE*100/3, 100);
      
      lightLocalizer = new LightLocalization();
      //Move to (1,1) using LS correction
      lightLocalizer.odoCorrectionFirst();
      
    //Traverse the second line (x=1) to use the odo correction method 
      NavigatorUtility.moveDistFwd((int) TILE_SIZE*100/4, 100);

      lightLocalizer.odoCorrectionSecond();
      
      NavigatorUtility.turnBy(-90);
      odometer.setXyt(TILE_SIZE, TILE_SIZE,0.0);
            
      //navigate tunnel
      navigation.navigateTunnel(3, 2, 5, 3,"Red ");
      
      //Add search and avoid obstacles
      
       sleepFor(1000);
    }
  }
  
  /**
   * Sleep the program for a given duration of time
   * @param duration to sleep in milliseconds
   * 
   */
  public static void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }
}
