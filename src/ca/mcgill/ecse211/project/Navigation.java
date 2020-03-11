package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;


/**
 * Class containing all methods used to navigate ev3 about the board, 
 * uses an instance of {@code LightLocalization} class to incorporate odometery correction.
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
   * Instance variable for LightLocalization
   */
  private LightLocalization lightLocalizer;
  
  /**
   * Buffer (array) to store US samples. Declared as an instance variable to avoid creating a new
   * array each time {@code readUsSample()} is called.
   */
  private static float[] usData = new float[usSensor.sampleSize()];
  
  /**
   * Array to hold odometer x, y, theta values
   */
  private static double odoValues[] = new double[3];  

/**
 * Constructor for navigation class, creates a Navigation object
 * @param lsLocalizer instance of LightLocalization class
 */
  public Navigation(LightLocalization lsLocalizer) {
    this.lightLocalizer = lsLocalizer;
  }
   
 /**
  * Calculates the minimum rotation angle needed to turn to target angle and turns ev3 by that minimum angle. 
  * The rotation angle is calculated by subtracting the current odometer angle from {@code theta} and ensuring that the rotation angle is below 180 degrees. 
  * Uses {@code NavigatorUtitlity} to convert rotation angle to total rotation of each wheel and rotates each wheel by that amount.
  * @param theta Target angle to turn to
  */
 public  void turnTo(double theta) {
   double rotationAngle = theta - odoValues[2];
   rotationAngle = Math.toDegrees(rotationAngle);
   
   ///ensures shortest angle is used
   if(rotationAngle > 180) {
     rotationAngle = 360 - rotationAngle;
   }else if(rotationAngle < -180) {
     rotationAngle = 360 + rotationAngle;
   }
   Sound.beep();
   int nav_turn_error = 1;
   // correct the heading
   leftMotor.rotate(NavigatorUtility.convertAngle(rotationAngle + nav_turn_error),true);
   rightMotor.rotate(-NavigatorUtility.convertAngle(rotationAngle + nav_turn_error),false);
 }
 
 /**
  * Moves the robot to provided grid coordinate. Calculates the distance to move and the angle to turn to using the provided grid coordinates 
  * and the current odometer values. Keeps moving the robot until travelled distance is less than the calculated distance.
  * @param x : x-coordinate of grid destination
  * @param y : y-coordinate of grid destination
  */
 public void travelTo(double x, double y) {
   //convert map coordinates to centimetres
    x = x * TILE_SIZE;
    y = y * TILE_SIZE;
    
    odoValues = odometer.getXyt();
    
    //subtracting odometer  x and y values to estimate x and y distance needed
    double distX = x - odoValues[0]; 
    double distY = y - odoValues[1];
    
    double theta = Math.atan2(distX, distY);
    
    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    turnTo(theta);
    
    double hypotenuse = Math.sqrt(distY*distY  + distX*distX);
    double currX = odoValues[0];
    double currY = odoValues[1];
    double distanceTravelled = 0;
    
    while( distanceTravelled <= hypotenuse ) {
      leftMotor.setSpeed(MOTOR_LOW);
      rightMotor.setSpeed(MOTOR_LOW);
      rightMotor.forward();
      leftMotor.forward();
      
      double xNew =  odometer.getXyt()[0];
      double yNew = odometer.getXyt()[1];
      distanceTravelled = Math.sqrt((xNew-currX)*(xNew-currX) + (yNew-currY)*(yNew-currY));

    }
  rightMotor.stop(true);
  leftMotor.stop(true);
 }
 
 /**
  * Navigates the robot across a tunnel and stops either at {@code (UR_X+1,(LL_Y+UR_Y)/2)} if tunnel is parallel to orgin x axis
  *  at {@code ((LL_X+UR_X)/2, UR_Y + 1)} if tunnel is parallel to y-axis. Uses {@code travelTo} to position the robot perpendicular to 
  * the tunnel opening. Uses {@code LightLocalization} to ensure heading of robot enters tunnel at the correct heading.  
  * Uses {@code travelTo} to move in a straight line through the tunnel and move 1 tile after leaving the tunnel.
  * @param LL_X the x coordinate of the lower left point of the tunnel
  * @param LL_Y the y coordinate of the lower left point of the tunnel
  * @param UR_X the x coordinate of the upper right point of the tunnel
  * @param UR_Y the y coordinate of the upper right point of the tunnel
  * @param team the team assigned to the ev3(red or green)
  */
public void navigateTunnel(double LL_X, double LL_Y, double UR_X, double UR_Y, String team) {

double currY = odometer.getY();
double currX = odometer.getX();

if(team .equals("Red")){ //red team
  if( Math.abs(UR_Y * TILE_SIZE - currY) < Math.abs(LL_Y * TILE_SIZE  - currY)){
    travelTo(LL_X-1, UR_Y);
   }else {
    travelTo(LL_X-1, LL_Y);
   }
  double currAngle = odometer.getAngle();
  currAngle = (currAngle * 180.0 / 3.14159);
  NavigatorUtility.turnBy(-1*currAngle);
  //moveStraightFor(0.25);
  lightLocalizer.odoCorrectionFirst();
  lightLocalizer.odoCorrectionSecond();
  travelTo(LL_X-1, (LL_Y+UR_Y)/2);
  travelTo(UR_X+1,(LL_Y+UR_Y)/2);
  }else { //green team
    travelTo(UR_X, LL_Y-1);
    double currAngle = odometer.getAngle();
    NavigatorUtility.turnBy(-1*currAngle);
    lightLocalizer.odoCorrectionFirst();
    lightLocalizer.odoCorrectionSecond();
    travelTo((LL_X+UR_X)/2, LL_Y-1);
    travelTo((LL_X+UR_X)/2, UR_Y + 1);
  }
}


 /**
  * Returns the filtered distance between the US sensor and an obstacle in cm.
  * 
  * @return the filtered distance between the US sensor and an obstacle in cm
  */
 private int readUsDistance() {
   usSensor.fetchSample(usData, 0);
   // extract from buffer, convert to cm, cast to int, and filter
   return filter((int) (usData[0] * 100.0));
 }
 
 /**
  * Rudimentary filter - toss out invalid samples corresponding to null signal.
  * 
  * @param distance raw distance measured by the sensor in cm
  * @return the filtered distance in cm
  */
 private int filter(int distance) {
   if (distance >= 255 && invalidSampleCount < INVALID_SAMPLE_LIMIT) {
     // bad value, increment the filter value and return the distance remembered from before
     invalidSampleCount++;
     return prevDistance;
   } else {
     if (distance < 255) {
       // distance went below 255: reset filter and remember the input distance.
       invalidSampleCount = 0;
     }
     prevDistance = distance;
     return distance;
   }
 } 
}