package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import static ca.mcgill.ecse211.project.Resources.*;

/**
 * Main class to launch the program and acts as controller. Creates a {@code UltarsonicLocalization} object which 
 * provides methods to localize to (1,1) relative to starting position on the grid. Starts the {@code Odometer} thread and the {@code Display} thread.
 * After initial localization, the {@code LightSensorPoller} thread is started and an {@code LightLocalization} object is created which is used for 
 * correction of the odometer. An instance of the {@code Navigation} class is then created to start of the navigation process. When the tunnel 
 * navigation is done, an instance of {@code ObjectDetection} class is created which provides methods for the search process.
 *
 * @author Mustafain , Bruno
 */
public class Main {
  /**
   * Instance of UltrasoncicLocalization Class, used for initial localization
   */
  private static UltrasonicLocalization usLocalizer;
  /**
   *  Instance of LightLocalization Class, used for odometer correction 
   */
  private static LightLocalization lightLocalizer;
  /**
   * Instance of Navigation class, provides acces to methods used for navigation of ev3
   */
  private static Navigation navigation;

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
      new Thread(new Display()).start();
      usLocalizer = new UltrasonicLocalization();
      usLocalizer.doLocalization();
  
      new Thread(new LightSensorPoller()).start();
      //Traverse the first line (x=1) to use the odo correction method 
      lightLocalizer = new LightLocalization();
      navigation = new Navigation(lightLocalizer);
     
      
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
      navigation.navigateTunnel(2, 2, 3, 4,"Green");
      
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
