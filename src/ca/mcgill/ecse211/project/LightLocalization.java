package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.DEG_PER_RAD;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.rightMotor;

/**
 * This class will provide methods to perform odometer correction using 
 * light sensors which are sampled by the {@link LightSensorPoller} thread.
 * In order to correct the position of the robot, this class will provide
 * methods to localize at different waypoints using two light sensors. 
 * The {@code odoCorrectionFirst} method corrects the robot's position
 * along the y-axis. The {@code odoCorrectionSecond} method corrects the 
 * robot's position along the x-axis.
 */
public class LightLocalization {

  /** 
   * Distance between sensors and front axle.
   */
  private int fow = 150;
  
  /** 
   * Pull Back on the wheel to clear the line to ensure consistent trials.
   */
  private int pullBack = -50;

  /** 
   * Testing parameter for shifting the final orientation.
   */
  private int lineThicknessCorrection = 0;

  /**
   * Creates a LightLocalization object.
   */
  public LightLocalization() { 
  }

  /**
   * Initial LS localization to (1,1) after US localization.
   */
  public void initialLocalizationUsingLS() {

    // Move past first line
    NavigatorUtility.moveDistFwd((int) TILE_SIZE * 100 / 3, 100);

    odoCorrectionFirst();

    // Traverse the second line (x=1) to use the odo correction method
    NavigatorUtility.moveDistFwd((int) TILE_SIZE * 100 / 4, 100);
    
    odoCorrectionSecond();
    sleepFor(3000);

    // Rotate back to suitable bearing, turn 270 degrees
    NavigatorUtility.turnBy(270);
  }

  /**
   * Light sensor correction along a single line used before and after the 
   * tunnel to ensure straight line travel through said tunnel. Uses static 
   * methods from {@code NavigatorUtility} class.
   * 
   * @param turnBy initial angle to turn by to align axle along the line we use to correct with
   */
  public void singleLineCorrection(double turnBy) {
    NavigatorUtility.turnBy(turnBy);
    NavigatorUtility.moveDistFwd(250, 100);

    while (true) {

      if (LightSensorPoller.getRightColorVal() <= 30 && LightSensorPoller.getLeftColorVal() <= 30) {
        rightMotor.stop(true);
        leftMotor.stop(true);
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      }

      if (LightSensorPoller.getRightColorVal() <= 30) {
        rightMotor.stop(true);
        leftMotor.rotate(-pullBack);
        leftMotor.backward();

        while (LightSensorPoller.getLeftColorVal() > 30) {
        }
        leftMotor.stop();
        leftMotor.rotate(-lineThicknessCorrection);

        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      
      } else if (LightSensorPoller.getLeftColorVal() <= 30) {
        leftMotor.stop(true);
        rightMotor.rotate(-pullBack);
        rightMotor.backward();

        while (LightSensorPoller.getRightColorVal() > 30) {
        }
        rightMotor.stop(true);
        rightMotor.rotate(-lineThicknessCorrection);

        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      } else { // if no line is detected keep moving backwards
        leftMotor.setSpeed(100);
        rightMotor.setSpeed(100);
        leftMotor.backward();
        rightMotor.backward();
      }
    }
  }


  /**
   * Corrects the EV3 along the the axis the robot is closest too. 
   * This is the first part of the light correction of the odometer.
   * The robot drives backwards until one or both of the light sensors 
   * cross a line, from this it aligns itself
   * along the line and turns 90 degrees clockwise.
   * 
   */
  public void odoCorrectionFirst() {
    while (true) {

      if (LightSensorPoller.getRightColorVal() <= 30 && LightSensorPoller.getLeftColorVal() <= 30) {
        rightMotor.stop(true);
        leftMotor.stop(true);
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        NavigatorUtility.turnBy(90); // turn by 90 degrees
        break;
      }


      if (LightSensorPoller.getRightColorVal() <= 30) {
        rightMotor.stop(true);
        leftMotor.rotate(-pullBack);
        leftMotor.backward();


        while (LightSensorPoller.getLeftColorVal() > 30) {
        }
        leftMotor.stop();
        leftMotor.rotate(-lineThicknessCorrection);

        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        NavigatorUtility.turnBy(90);
        break;
      } else if (LightSensorPoller.getLeftColorVal() <= 30) {
        leftMotor.stop(true);
        rightMotor.rotate(-pullBack);
        rightMotor.backward();

        while (LightSensorPoller.getRightColorVal() > 30) {
        }
        rightMotor.stop(true);
        rightMotor.rotate(-lineThicknessCorrection);

        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        NavigatorUtility.turnBy(90);
        break;
      } else { // if no line is detected keep moving backwards
        leftMotor.setSpeed(100);
        rightMotor.setSpeed(100);
        leftMotor.backward();
        rightMotor.backward();
      }
    }
  }

  /**
   * Corrects the EV3 along the the second axis. 
   * This is the second part of the light correction of the odometer.
   * The robot drives backwards until one or both of the light sensors 
   * cross a line, from this it aligns itself along the
   * line. The robot now sits centered above a point on the grid.
   */
  public void odoCorrectionSecond() {
    // Go over line to then move backwards into it
    NavigatorUtility.moveDistFwd(350, 100);

    while (true) {

      if (LightSensorPoller.getRightColorVal() <= 30 && LightSensorPoller.getLeftColorVal() <= 30) {
        rightMotor.stop(true);
        leftMotor.stop(true);
        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      }


      if (LightSensorPoller.getRightColorVal() <= 30) {
        rightMotor.stop(true);
        leftMotor.rotate(pullBack);
        leftMotor.forward();

        while (LightSensorPoller.getLeftColorVal() > 30) {
        }
        leftMotor.stop();
        leftMotor.rotate(lineThicknessCorrection);

        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      } else if (LightSensorPoller.getLeftColorVal() <= 30) {
        leftMotor.stop(true);
        rightMotor.rotate(pullBack);
        rightMotor.forward();

        while (LightSensorPoller.getRightColorVal() > 30) {
        }
        rightMotor.stop(true);
        rightMotor.rotate(lineThicknessCorrection);

        sleepFor(1000);
        NavigatorUtility.moveDistFwd(fow, 100);
        break;
      } else { // if no line is detected keep moving backwards
        leftMotor.setSpeed(100);
        rightMotor.setSpeed(100);
        leftMotor.backward();
        rightMotor.backward();

      }
    }
  }

  /**
   * This method updates the odometer coordinates to those of the point
   * that the {@code odoCorrection} just localized to. It sets them
   * to the point to which the robot thinks it is currently closest to.
   */
  public void updateOdometer() {
    // Correct coordinates
    odometer.setX(closestTile(odometer.getXyt()[0]));
    odometer.setY(closestTile(odometer.getXyt()[1]));

    double currentOdoTheta = odometer.getXyt()[2];

    // Correct theta
    if (currentOdoTheta < 100 / DEG_PER_RAD && currentOdoTheta > 80 / DEG_PER_RAD) {
      odometer.setTheta(90 / Resources.DEG_PER_RAD);
    } else {
      odometer.setTheta(270 / Resources.DEG_PER_RAD);
    }
  }

  /**
   * Method to find closest tile.
   * 
   * @param odo location that robot thinks it is at
   * @return closestTile
   */
  private double closestTile(double odo) {
    int closest = (int) ((odo / TILE_SIZE) + 0.5); // + 0.5 so (int) cast rounds to nearest int
    return closest * TILE_SIZE;
  }

  /**
   * Sleep the program for a given duration of time.
   * 
   * @param duration The sleep duration in milliseconds
   */
  private void sleepFor(long duration) {
    try {
      Thread.sleep(duration);
    } catch (InterruptedException e) {
      // There is nothing to be done here
    }
  }

}
