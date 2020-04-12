package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.BASE_WIDTH;
import static ca.mcgill.ecse211.project.Resources.WHEEL_RAD;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.rightMotor;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * The odometer class keeps track of the robot's (x, y, theta) position.
 */
public class Odometer implements Runnable {

  /**
   * The x-axis position in cm.
   */
  private volatile double x;

  /**
   * The y-axis position in cm.
   */
  private volatile double y;

  /**
   * The orientation in degrees.
   */
  private volatile double theta; // Heading angle

  /**
   * The (x, y, theta) position as an array.
   */
  private double[] position;

  // Thread control tools
  /**
   * Fair lock for concurrent writing.
   */
  private static Lock lock = new ReentrantLock(true);

  /**
   * Indicates if a thread is trying to reset any position parameters.
   */
  private volatile boolean isResetting = false;

  /**
   * Lets other threads know that a reset operation is over.
   */
  private Condition doneResetting = lock.newCondition();

  private static Odometer odo; // Returned as singleton

  // Motor-related variables
  private static int leftMotorTachoCount = 0;
  private static int rightMotorTachoCount = 0;

  /**
   * The odometer update period in ms.
   */
  private static final long ODOMETER_PERIOD = 25;

  private int lastTachoL; // Last tacho counts for L and R moors
  private int lastTachoR;

  /**
   * This is the default constructor of this class. 
   * It initiates all motors and variables once. 
   * It cannot be accessed externally.
   */
  private Odometer() {
    setXyt(0, 0, 0);
    lastTachoL = leftMotor.getTachoCount(); // Get initial tacho counts
    lastTachoR = rightMotor.getTachoCount();
  }

  /**
   * Returns the Odometer Object. Use this method to obtain an instance of Odometer.
   * 
   * @return the Odometer Object
   */
  public static synchronized Odometer getOdometer() {
    if (odo == null) {
      odo = new Odometer();
    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run.
   * 
   * <p>It estimates the left and right wheel displacements using tachometer counts. 
   * The change in angle is then calculated using these estimated displacements 
   * using the wheel base and wheel radius of the robot.
   */
  public void run() {

    while (true) {
      long updateStart = System.currentTimeMillis();

      leftMotorTachoCount = leftMotor.getTachoCount();
      rightMotorTachoCount = rightMotor.getTachoCount();

      // Calculate new robot position based on tachometer counts
      // compute L and R wheel displacements:
      double distL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - lastTachoL) / 180;
      double distR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - lastTachoR) / 180;
      lastTachoL = leftMotorTachoCount;
      lastTachoR = rightMotorTachoCount;
      double deltaD = 0.5 * (distL + distR); 
      double deltaT = (distL - distR) / BASE_WIDTH; // compute change in heading
      setTheta(this.theta + deltaT);
      double dX = deltaD * Math.sin(theta); // compute X component of displacement
      double dY = deltaD * Math.cos(theta); // compute Y component of displacement

      // Update odometer values with new calculated values using update()
      setX(x + dX);
      setY(y + dY);
      // this ensures that the odometer only runs once every period
      long updateDuration = System.currentTimeMillis() - updateStart;
      if (updateDuration < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - updateDuration);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
    }
  }

  /**
   * Gets the tacho count for each motor.
   * 
   * @return the array containing the tacho count for left and right motors.
   */
  public int[] getTachos() {
    return new int[] {lastTachoL, lastTachoR};
  }

  // IT IS NOT NECESSARY TO MODIFY ANYTHING BELOW THIS LINE

  /**
   * Returns the Odometer data.
   * 
   * <p>Writes the current position and orientation of the robot onto the odoData array.
   * {@code odoData[0] = x, odoData[1] = y; odoData[2] = theta;}
   * 
   * @return the odometer data.
   */
  public double[] getXyt() {
    double[] position = new double[3];
    lock.lock();
    try {
      while (isResetting) { // If a reset operation is being executed, wait until it is over.
        doneResetting.await(); // Using await() is lighter on the CPU than simple busy wait.
      }

      position[0] = x;
      position[1] = y;
      position[2] = theta;
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      lock.unlock();
    }
    return position;
  }

  /**
   * Returns the x-axis position.
   * 
   * @return the odometer x position
   */
  public double getX() {
    return getXyt()[0];
  }

  /**
   * Returns the y-axis position.
   * 
   * @return the odometer y position
   */
  public double getY() {
   
    return getXyt()[1];
  }

  /**
   * Adds dx, dy and dtheta to the current values of x, y and theta, respectively. 
   * Useful for odometry.
   * 
   * @param dx the change in x
   * @param dy the change in y
   * @param dtheta the change in theta
   */
  public void update(double dx, double dy, double dtheta) {
    lock.lock();
    isResetting = true;
    try {
      x += dx;
      y += dy;
      if (dtheta < 0) {
        dtheta += 360;
      }
      theta = ((theta + dtheta) % 360); // keeps the updates within 360 degrees
      isResetting = false;
      doneResetting.signalAll(); // Let the other threads know we are done resetting
    } finally {
      lock.unlock();
    }

  }

  /**
   * Overrides the values of x, y and theta. Use for odometry correction.
   * 
   * @param x the value of x
   * @param y the value of y
   * @param theta the value of theta in degrees
   */
  public void setXyt(double x, double y, double theta) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      this.y = y;
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites x. Use for odometry correction.
   * 
   * @param x the value of x
   */
  public void setX(double x) {
    lock.lock();
    isResetting = true;
    try {
      this.x = x;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Gets the current heading angle of the robot recorded by the odometer.
   * 
   * @return theta Current heading/orientation of robot
   */
  public double getAngle() {
    return this.theta;
  }

  /**
   * Overwrites y. Use for odometry correction.
   * 
   * @param y the value of y
   */
  public void setY(double y) {
    lock.lock();
    isResetting = true;
    try {
      this.y = y;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

  /**
   * Overwrites theta. Use for odometry correction.
   * 
   * @param theta the value of theta
   */
  public void setTheta(double theta) {
    lock.lock();
    isResetting = true;
    while (theta >= 2 * Math.PI) {
      theta -= 2 * Math.PI;
    }
    while (theta < 0) {
      theta += 2 * Math.PI;
    }
    try {
      this.theta = theta;
      isResetting = false;
      doneResetting.signalAll();
    } finally {
      lock.unlock();
    }
  }

}
