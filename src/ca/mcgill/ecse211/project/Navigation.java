package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.MOTOR_LOW;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import static ca.mcgill.ecse211.project.Resources.usSensor;


/**
 * Class containing all methods used to navigate EV3 about the board, 
 * uses an instance of {@link LightLocalization}
 * class to incorporate odometry correction.
 * 
 * @author Mustafain, Bruno
 *
 */
public class Navigation {
  /**
   * The distance remembered by the {@code filter()} method.
   */
  private int prevDistance;

  /**
   * The number of invalid samples seen by {@code filter()} so far.
   */
  private int invalidSampleCount;

  /**
   * Instance variable for LightLocalization.
   */
  private LightLocalization lightLocalizer;

  /**
   * Buffer (array) to store US samples. Declared as an instance 
   * variable to avoid creating a new array each time 
   * {@code readUsSample()} is called.
   */
  private static float[] usData = new float[usSensor.sampleSize()];

  /**
   * Array to hold odometer values for x, y, theta.
   */
  private static double[] odoValues = new double[3];

  /**
   * Constructor for navigation class, creates a Navigation object.
   * 
   * @param lsLocalizer instance of LightLocalization class
   */
  public Navigation(LightLocalization lsLocalizer) {
    this.lightLocalizer = lsLocalizer;
  }

  /**
   * Calculates the minimum rotation angle needed to turn to target angle 
   * and turns the EV3 by that minimum angle. The rotation angle is 
   * calculated by subtracting the current odometer angle from {@code theta} 
   * and ensuring that the rotation angle is below 180 degrees. 
   * Uses {@code NavigatorUtitlity} to convert rotation angle to total rotation 
   * of each wheel and rotates each wheel by that amount.
   * 
   * @param theta Target angle to turn to
   */
  public void turnTo(double theta) {
    double rotationAngle = theta - odoValues[2];
    rotationAngle = Math.toDegrees(rotationAngle);

    // ensures shortest angle is used
    if (rotationAngle > 180) {
      rotationAngle = 360 - rotationAngle;
    } else if (rotationAngle < -180) {
      rotationAngle = 360 + rotationAngle;
    }
    
    //angle error while turning
    int nav_turn_error = 1;
    // correct the heading
    leftMotor.rotate(NavigatorUtility.convertAngle(rotationAngle + nav_turn_error), true);
    rightMotor.rotate(-NavigatorUtility.convertAngle(rotationAngle + nav_turn_error), false);
  }

  /**
   * Moves the robot to provided grid coordinate. Calculates the distance to 
   * move and the angle to turn to using the provided grid coordinates and 
   * the current odometer values. Keeps moving the robot until traveled 
   * distance is less than the calculated distance.
   * 
   * @param x : x-coordinate of grid destination
   * @param y : y-coordinate of grid destination
   * @param speed : speed to use while traveling
   */
  public void travelTo(double x, double y, int speed) {
    // convert map coordinates to centimetres
    x *= TILE_SIZE;
    y *= TILE_SIZE;

    odoValues = odometer.getXyt();

    // subtracting odometer x and y values to estimate x and y distance needed
    double distX = x - odoValues[0];
    double distY = y - odoValues[1];

    double theta = Math.atan2(distX, distY);

    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    turnTo(theta);

    double hypotenuse = Math.sqrt(distY * distY + distX * distX);
    double currX = odoValues[0];
    double currY = odoValues[1];
    double distanceTravelled = 0;

    while (distanceTravelled <= hypotenuse) {
      leftMotor.setSpeed(speed);
      rightMotor.setSpeed(speed);
      rightMotor.forward();
      leftMotor.forward();

      double xNew = odometer.getXyt()[0];
      double yNew = odometer.getXyt()[1];
      distanceTravelled = Math.sqrt((xNew - currX) * (xNew - currX) 
                                      + (yNew - currY) * (yNew - currY));

    }
    rightMotor.stop(true);
    leftMotor.stop(true);
  }

  /**
   * Navigates the robot across a tunnel and stops either at 
   * {@code (UR_X+1,(LL_Y+UR_Y)/2)} if tunnel is parallel to
   * origin x axis, or at {@code ((LL_X+UR_X)/2, UR_Y + 1)} if 
   * tunnel is parallel to y-axis. Uses {@code travelTo} to
   * position the robot perpendicular to the tunnel opening. 
   * Uses {@code LightLocalization} to ensure heading of robot
   * enters tunnel at the correct heading. Uses {@code travelTo} 
   * to move in a straight line through the tunnel and move
   * 1 tile after leaving the tunnel.
   * 
   * @param LL_X the x coordinate of the lower left point of the tunnel
   * @param LL_Y the y coordinate of the lower left point of the tunnel
   * @param UR_X the x coordinate of the upper right point of the tunnel
   * @param UR_Y the y coordinate of the upper right point of the tunnel
   * @param team the team assigned to the ev3(red or green)
   */
  public void navigateTunnel(double LL_X, double LL_Y, double UR_X, double UR_Y, String team) {

    double currY = odometer.getY();
    double currX = odometer.getX();

    if (team.equals("Red")) { // red team
    	//travel 1 tile away from tunnel opening
      if (Math.abs(UR_Y * TILE_SIZE - currY) < Math.abs(LL_Y * TILE_SIZE - currY)) {
        travelTo(LL_X - 1, UR_Y, MOTOR_LOW);
      } else {
        travelTo(LL_X - 1, LL_Y, MOTOR_LOW);
      }
      double currAngle = odometer.getAngle();
      currAngle = Math.toDegrees(currAngle);
      NavigatorUtility.turnBy(-1 * currAngle);
      lightLocalizer.odoCorrectionFirst();
      lightLocalizer.odoCorrectionSecond();
      //travel in a straight line through the tunnel
      travelTo(LL_X - 1, (LL_Y + UR_Y) / 2, MOTOR_LOW);
      lightLocalizer.singleLineCorrection(90);
      travelTo(UR_X + 1, (LL_Y + UR_Y) / 2, 350);
    } else { // green team
    	//travel 1 tile away from tunnel opening
      travelTo(UR_X, LL_Y - 1, MOTOR_LOW);
      double currAngle = odometer.getAngle();
      currAngle = Math.toDegrees(currAngle);
      NavigatorUtility.turnBy(-1 * currAngle);
      lightLocalizer.odoCorrectionFirst();
      lightLocalizer.odoCorrectionSecond();
    //travel in a straight line through the tunnel
      travelTo((LL_X + UR_X) / 2, LL_Y - 1, MOTOR_LOW);
      lightLocalizer.singleLineCorrection(90);
      travelTo((LL_X + UR_X) / 2, UR_Y + 1, 350);
    }
  }
}
