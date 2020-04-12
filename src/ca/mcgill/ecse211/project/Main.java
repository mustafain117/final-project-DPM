package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.szr;
import static ca.mcgill.ecse211.project.Resources.tnr;

import lejos.hardware.Button;
import lejos.hardware.Sound;

/**
 * Main class to launch the program and acts as controller. 
 * Creates a {@code UltarsonicLocalization} object which
 * provides methods to localize to (1,1) relative to starting 
 * position on the grid. Starts the {@code Odometer} thread
 * and the {@code Display} thread. After initial localization, 
 * the {@code LightSensorPoller} thread is started and a
 * {@code LightLocalization} object is created which is used 
 * for correction of the odometer. An instance of the
 * {@code Navigation} class is then created to start of the 
 * navigation process. When the tunnel navigation is done, an
 * instance of {@code ObjectDetection} class is created which 
 * provides methods for the search process.
 *
 * @author Mustafain , Bruno
 */
public class Main {
  /**
   * Instance of UltrasoncicLocalization Class, used for initial localization.
   */
  private static UltrasonicLocalization usLocalizer;
  
  /**
   * Instance of LightLocalization Class, used for odometer correction.
   */
  private static LightLocalization lightLocalizer;
  
  /**
   * Instance of Navigation class, provides access to methods used for navigation of EV3.
   */
  private static Navigation navigation;

  /**
   * Main method starts up the odometer and LightSensorPoller threads, 
   * creates an instance of UltrasonicLocalization to
   * localize and starts navigation and search process.
   * 
   * @param args Array of command line params
   */
  public static void main(String[] args) {
    // wait for user button left press
    Display.test();
    int buttonChoice;
    buttonChoice = Button.waitForAnyPress();

    // Press ENTER to start the full code from start to finish
    if (buttonChoice == Button.ID_ENTER) {
      // Setup
      new Thread(odometer).start();
      new Thread(new Display()).start();
      new Thread(new LightSensorPoller()).start();
      lightLocalizer = new LightLocalization();
      navigation = new Navigation(lightLocalizer);
      
      // Start US localization
      usLocalizer = new UltrasonicLocalization();
      usLocalizer.doLocalization();

      // Move to (1,1) using LS correction
      lightLocalizer.initialLocalizationUsingLS();

      // Set Initial bearings
      // TODO depending on what corner is started in, 
      // different initial positions and bearings must be set
      odometer.setXyt(TILE_SIZE, TILE_SIZE, 0.0);
      
      //beep 3 times to indicate end of initial localization
      for(int i = 0 ; i < 3 ; i++) {
    	  Sound.beep();
    	  sleepFor(100);
      }

      // navigate tunnel
      // TODO determine tunnel parameters depending on starting position
      navigation.navigateTunnel(tnr.ll.x, tnr.ll.y, tnr.ur.x, tnr.ur.y, "Red");

      // Add search and avoid obstacles
      RobotClaw claw = RobotClaw.getClaw();
      ObjectDetection detector = new ObjectDetection(usLocalizer, navigation, claw, lightLocalizer);
      // TODO determine parameters depending on wifi parameters
      detector.searchVehicle(szr.ll.x, szr.ll.y, szr.ur.x, szr.ur.y);


      // TODO return to start,assume the search vehicle terminates with 
      // the robot back at the same place it started (in front of tunnel)

    }
  }

  /**
   * Sleep the program for a given duration of time.
   * 
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
