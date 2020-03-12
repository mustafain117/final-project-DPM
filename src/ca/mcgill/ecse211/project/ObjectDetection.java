package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.INVALID_SAMPLE_LIMIT;
import static ca.mcgill.ecse211.project.Resources.MOTOR_LOW;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.rightMotor;
import static ca.mcgill.ecse211.project.Resources.usSensor;
import lejos.hardware.Sound;

/**
 * Class to implement search and obstacle avoidance. The {@code searchVehicle} method starts the search process.. It takes in the search zone
 *  boundary parameters and generates a set of way points to travel to. It calls the {@code moveToWaypoint} method to move to a specific way
 *  point and also checks if there are any objects in its path. If any object is detected, it is checked if its an obstacle or a vehicle. 
 *  In case of an obstacle, the {@code avoidObstacle} method is called to avoid the obstacle. 
  * In case of the vehicle, the {@code towVehicle} method is called to attach the vehicle to the claw.
 * @author Mustafain
 */
public class ObjectDetection {
  
  private static final int SAFE_DIST = 5;

  /**
   * The number of invalid samples seen by {@code filter()} so far.
   */
  private int invalidSampleCount;
  
  /**
   * Array to contain the data returned by reading the ultrasonic sensor.
   */
  private float[] usData = new float[usSensor.sampleSize()];

  /**
   * The distance remembered by the {@code filter()} method.
   */
  private int prevDistance;

  /**
   * Field for instance of navigation class
   */
  private Navigation navigation;

  /**
   *  Instance of RobotClaw class;
   */
  private RobotClaw claw;

  /**
   * Constructor for ObjectDetection Class
   * @param navigation : Instance of Navigation Class
   * @param claw : object representing the claw of the robot
   */
  public ObjectDetection(Navigation navigation, RobotClaw claw) {
    this.navigation = navigation;
    this.claw = claw;
  }
  
 /**
  * This method starts the vehicle search process. It takes in the search zone boundary parameters and generates a set of way points to travel to.
  * It calls the {@code moveToWaypoint} method to move to a specific way point and also checks if there are any objects in its path. If any object is
  * detected, it is checked if its an obstacle or a vehicle. In case of an obstacle, the {@code avoidObstacle} method is called to avoid the obstacle. 
  * In case of the vehicle, the {@code towVehicle} method is called to attach the vehicle to the claw.
  * @param SZ_LL_X the lower left x coordinate of the assigned search zone
  * @param SZ_LL_Y the lower left y coordinate of the assigned search zone
  * @param SZ_UR_X the upper right x coordinate of the assigned search zone
  * @param SZ_UR_Y the upper right y coordinate of the assigned search zone
  */
  public void searchVehicle(double SZ_LL_X, double SZ_LL_Y, double SZ_UR_X, double SZ_UR_Y) {
    
  }
  
  /**
   * This method is called when an obstacle is detected. It avoids the obstacle by moving around the obstacle from the left or right directions
   * depending on the current position of the robot and the search zone boundary.
   */
  public void avoidObstacle() {
    
  }
  
  public void moveToWaypoint(double x , double y) {
  //convert map coordinates to centimetres
    x = x * TILE_SIZE;
    y = y * TILE_SIZE;
    
    double[] odoValues = odometer.getXyt();
    //subtracting odometer  x and y values to estimate x and y distance needed
    double distX = x - odoValues[0]; 
    double distY = y - odoValues[1];
    
    double theta = Math.atan2(distX, distY);
    
   // Sound.beep();
    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    navigation.turnTo(theta);
    
    double hypotenuse = Math.sqrt(distY*distY  + distX*distX);
    double currX = odoValues[0];
    double currY = odoValues[1];
    double distanceTravelled = 0;
 
    while(distanceTravelled <= hypotenuse ) {
      leftMotor.setSpeed(MOTOR_LOW);
      rightMotor.setSpeed(MOTOR_LOW);
      rightMotor.forward();
      leftMotor.forward();

      if(readUsDistance() < SAFE_DIST) {
        rightMotor.stop(true);
        leftMotor.stop(true);
        Sound.beep();
        Sound.beep();
        if(isObstacle()) {
          avoidObstacle();
          return;
        }else {
          //vehicle detected, begin rescue process
           return;
        }
      } 

      double xNew =  odometer.getX();
      double yNew = odometer.getY();
      distanceTravelled = Math.sqrt((xNew-currX)*(xNew-currX) + (yNew-currY)*(yNew-currY));
    }
      rightMotor.stop();
      leftMotor.stop();
  }
  
  /**
   * This method is used to characterise a detected object. It returns true if the detected object is an obstacle, returns false otherwise.
   * @return true if obstacle is detected, false otherwise.
   */
  private boolean isObstacle() {
    // TODO Auto-generated method stub
    return false;
  }

  /**
   * This method is used to detect the axle of the stranded vehicle using the color detected by the color sensor from the LightSensorPoller thread.
   * It is called when a vehicle is detected, this method orients the claw to be in line with the axle. The {@code RobotClaw} class is then used to 
   * attach the vehicle to the robot. 
   */
  public void towVehicle() {
  }
  
  /**
   * Reads the ultrasonic sensor distance and calls the filter method.
   * 
   * @return filtered distance value read by ultrasonic sensor
   */
  private int readUsDistance() {
    usSensor.fetchSample(usData, 0);
    // extract from buffer, convert to cm, cast to int, and filter
    return filter((int) (usData[0] * 100.0)); // *100 for cm instead of m
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
        // minDistance = Math.min(minDistance, distance);
      }
      prevDistance = distance;
      return distance;
    }
  }
  
}
