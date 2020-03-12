package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.*;
import lejos.hardware.Sound;

/**
 * The UltrasonicLocalization class implements the ultrasonic localization techniques using
 * falling edge. Also implements movement to the point (1,1) using ultrasonic sensor
 */
public class UltrasonicLocalization {

  /**
   * The distance remembered by the {@code filter()} method.
   */
  private int prevDistance;

  /**
   * The number of invalid samples seen by {@code filter()} so far.
   */
  private int invalidSampleCount;

  /**
   * Error margin on ultrasonic sensor readings to counter noise.
   */
  private int errorMargin = 3;

  /**
   * Change in heading after localization finds the falling edges.
   */
  private double deltaT;

  /**
   * Angle to rotate to be displayed on LCD.
   */
  private double rotationAngle = 0.0;

  /**
   * Array to contain the data returned by reading the ultrasonic sensor.
   */
  private float[] usData = new float[usSensor.sampleSize()];

  /**
   * Constructor for the {@code UsLocalizer} class.
   *
   */
  public UltrasonicLocalization() {
  }

  /**
   * Performs localization process based on falling edge technique, localizes the heading to 0 degrees. Uses the ultrasonice sensor
   * to detect the edges of the wall and uses odometer theta values to calculate angle to turn to in order to localize heading to 0 degrees.
   */
  public void doLocalization() {
    double angleA;  // angle to first falling edge
    double angleB;  // angle to second falling edge

    // set the rotational speed of the motors slow to get more sensor data
    leftMotor.setSpeed(MOTOR_ROT);
    rightMotor.setSpeed(MOTOR_ROT);

      // represents case where robot starts off facing the wall; turn away from the wall
      while (readUsDistance() < INITIAL_DIST_THRESHOLD) {
        rotateClockwise();
      }
      // stop the motors
      rightMotor.stop();
      leftMotor.stop();
      odometer.setXyt(0,0,0);   // reset the odometer, then begin actual localization process
      // rotate until robot faces away from wall
      while (readUsDistance() < WALL_DIST + errorMargin) {
        rotateAntiClockwise();
      }
      // rotate until robot faces wall, record angle
      while (readUsDistance() > WALL_DIST) {
        rotateAntiClockwise();
      }
      angleA = convertAngle(odometer.getXyt()[2]);
      Sound.beep();
      // turn 90 degrees to avoid error where robot reads the two falling edges on the same wall 
      NavigatorUtility.turnBy(TURN_90);
      // switch direction and face wall
      while (readUsDistance() < WALL_DIST + errorMargin) {
        rotateClockwise();
      }

      // keep rotating until faces wall
      while (readUsDistance() > WALL_DIST) {
        rotateClockwise();
      }
      rightMotor.stop();    // stop motors, make a beep sound
      leftMotor.stop();
      Sound.beep();

      angleB = convertAngle(odometer.getXyt()[2]);

      // calculate angle to turn to correct heading to 0 degrees;
      // this formula is from the localization tutorial
      if (angleA > angleB) {
        deltaT = THETA_HIGH_FALLING - (angleA + angleB) / 2;
      } else {
        deltaT = THETA_LOW_FALLING - (angleA + angleB) / 2;
      }

      rotationAngle = deltaT + angleB;  // angle needed to rotate to 0 deg

      leftMotor.rotate(-getDistance(WHEEL_RAD, BASE_WIDTH, rotationAngle - TURNING_ERROR), true);
      rightMotor.rotate(getDistance(WHEEL_RAD, BASE_WIDTH, rotationAngle - TURNING_ERROR), false);
} 


  /**
   * Converts angle from degrees to radians.
   * 
   * @param angle Angle in degrees
   * @return equivalent angle in radians
   */
  private double convertAngle(double angle) {
    return (angle * 180 / Math.PI);
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

  /**
   * Rotates the robot in the clockwise direction by turning the 
   * left motor forward and the right motor backward.
   */
  private void rotateClockwise() {
    leftMotor.forward();
    rightMotor.backward();
  }

  /**
   * Rotates the robot in the counter-clockwise direction by turning 
   * the left motor backward and the right motor forward.
   */
  private void rotateAntiClockwise() {
    leftMotor.backward();
    rightMotor.forward();
  }

  /**
   * Gets the distance to rotate given the radius of the wheels,
   * the distance between the wheels, and the angle to turn.
   * 
   * @param radius Radius of the robot's wheels
   * @param width Distance between the two wheels
   * @param angle Angle to rotate the robot
   * @return distance int converted distance
   */
  private  int getDistance(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }

  /**
   * Converts the input radius and distance to an angle to rotate, based on the
   * formula {@code (arc length) = (radius) * (theta)}.
   * 
   * @param radius Radius of the robot's wheels
   * @param distance The length of the arc traced out by the turn
   * @return
   */
  private  int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * Uses the ultrasonic sensor to read the x and y distances that the robot is 
   * from the wall. These distances are used to calculate the distance that the 
   * robot needs to travel to get to (1,1).
   * 
   * @return double array containing x and y distances from the wall
   */
  public double[] recordDistances() {
    leftMotor.setSpeed(MOTOR_HIGH); // set left and right motor speeds high
    rightMotor.setSpeed(MOTOR_HIGH);
    NavigatorUtility.turnBy(TURN_180); // turn robot by 180 degrees
    double y;
    double x;
    y = readUsDistance(); // read y distance
    leftMotor.setSpeed(MOTOR_HIGH);
    rightMotor.setSpeed(MOTOR_HIGH);
    NavigatorUtility.turnBy(TURN_90); // turn robot by 90 degrees
    x = readUsDistance(); // read x distance
    NavigatorUtility.turnBy(TURN_90); // rotate 90 degrees and return to original 0 degree heading
    return new double[] {x, y};
  }

  /**
   * Moves the robot by the input distances {@code distX, distY}.
   * 
   * @param distX The distance to move in the x direction
   * @param distY The distance to move in the y direction
   */
  public void usNavigate(double distX, double distY) {
    leftMotor.setSpeed(MOTOR_ROT);
    rightMotor.setSpeed(MOTOR_ROT);

    leftMotor.rotate(convertDistance(WHEEL_RAD, distY - ROBOT_LENGTH), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, distY - ROBOT_LENGTH), false);

    NavigatorUtility.turnBy(TURN_90); // rotate by 90 degrees

    leftMotor.rotate(convertDistance(WHEEL_RAD, distX - ROBOT_LENGTH), true);
    rightMotor.rotate(convertDistance(WHEEL_RAD, distX - ROBOT_LENGTH), false);

    NavigatorUtility.turnBy(-TURN_90); // rotate by -90 degrees
  }

}