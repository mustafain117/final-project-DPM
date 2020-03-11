package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.BASE_WIDTH;
import static ca.mcgill.ecse211.project.Resources.DEG_PER_RAD;
import static ca.mcgill.ecse211.project.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.rightMotor;

/**
 * Helper class with static methods used by {@code Navigation}, {@code LightLocalization} and {@code UltrasonicLocalization} classes.
 * @author Mustafain, Bruno
 *
 */
public class NavigatorUtility {
  /**
   * Converts input distance to the total rotation of each wheel needed to cover that distance.
   * 
   * @param distance the input distance
   * @return the wheel rotations necessary to cover the distance
   */
  private static int convertDistance(double distance) {
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
