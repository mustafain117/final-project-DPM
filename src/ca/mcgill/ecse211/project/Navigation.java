package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import java.util.ArrayList;
import lejos.hardware.Sound;


/**
 * Class containing all methods used to move about the board
 * @author Mustafain & Bruno
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
   * Buffer (array) to store US samples. Declared as an instance variable to avoid creating a new
   * array each time {@code readUsSample()} is called.
   */
  private static float[] usData = new float[usSensor.sampleSize()];
  
  /**
   * array to hold odometer x, y, theta values
   */
  private static double odoValues[] = new double[3];  

   
 /**
  * Calculates the minimum angle needed to correct the robot's heading and turns by that angle.
  * @param theta Target angle to turn to
  */
 private static void turnTo(double theta) {
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
   leftMotor.rotate(convertAngle(rotationAngle + nav_turn_error),true);
   rightMotor.rotate(-convertAngle(rotationAngle + nav_turn_error),false);
 }
 
 /**
  * Moves the robot to provided grid coordinate
  * @param x : x-coordinate of grid destination
  * @param y : y-coordinate of grid destination
  */
 private static  void travelTo(double x, double y) {
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
  * navigates the robot across a tunnel and stops 1 tile after leaving tunnel end
  * @param LL_X the x coordinate of the lower left point of the tunnel
  * @param LL_Y the y coordinate of the lower left point of the tunnel
  * @param UR_X the x coordinate of the upper right point of the tunnel
  * @param UR_Y the y coordinate of the upper right poitn of the tunnel
  */
 public static  void navigateTunnel(double LL_X, double LL_Y, double UR_X, double UR_Y) {


travelTo(LL_X-1, LL_Y);
// turnTo(0);
double currAngle = odometer.getAngle();
currAngle = (currAngle * 180.0 / 3.14159);
turnBy(-1*currAngle);
//moveStraightFor(0.25);
LightLocalization.odoCorrectionFirst();
LightLocalization.odoCorrectionSecond();
travelTo(LL_X-1, (LL_Y+UR_Y)/2);

travelTo(UR_X+1,(LL_Y+UR_Y)/2);
}
 /**
  * Returns the filtered distance between the US sensor and an obstacle in cm.
  * 
  * @return the filtered distance between the US sensor and an obstacle in cm
  */
 public int readUsDistance() {
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
 int filter(int distance) {
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

 /**
  * Converts input distance to the total rotation of each wheel needed to cover that distance.
  * 
  * @param distance the input distance
  * @return the wheel rotations necessary to cover the distance
  */
 public static int convertDistance(double distance) {
   return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
 }

 /**
  * Converts input angle to the total rotation of each wheel needed to rotate 
  * the robot by that angle.
  * 
  * @param angle the input angle
  * @return the wheel rotations necessary to rotate the robot by the angle
  */
 public static int convertAngle(double angle) {
   return convertDistance(Math.PI * BASE_WIDTH * angle / 360.0);
 }

 /**
  * Turns the robot by a specified angle. Note that this method is different 
  * from {@code Navigation.turnTo()}. For example, if the robot is facing 90 
  * degrees, calling {@code turnBy(90)} will make the robot turn to 180 degrees, 
  * but calling {@code Navigation.turnTo(90)} should do nothing 
  * (since the robot is already at 90 degrees).
  * 
  * @param angle the angle by which to turn, in degrees
  */
 public static void turnBy(double angle) {
   leftMotor.setSpeed(150);
   rightMotor.setSpeed(150);
   leftMotor.rotate(convertAngle(angle), true);
   rightMotor.rotate(-convertAngle(angle), false);
 }
 
 
 /**
  * Rotates forward in a straight line for specified distance.
  * 
  * @param distance the distance
  * @param speed the speed in deg/s
  */
 public static void moveDistFwd(int distance, int speed) {
  int rotationAngle = (int) (distance * DEG_PER_RAD / 100); // Convert linear distance to turns
   leftMotor.setSpeed(speed); // Roll both motors forward
   rightMotor.setSpeed(speed);
   leftMotor.rotate(rotationAngle, true); // Rotate left motor - DO NOT BLOCK
   rightMotor.rotate(rotationAngle); // Rotate right motor
 }
 
}