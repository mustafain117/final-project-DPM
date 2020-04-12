package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.CLAW_ANGLE;
import static ca.mcgill.ecse211.project.Resources.MOTOR_LOW;
import static ca.mcgill.ecse211.project.Resources.TILE_SIZE;
import static ca.mcgill.ecse211.project.Resources.leftMotor;
import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.rightMotor;

import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.Region;
import ca.mcgill.ecse211.project.LightSensorPoller.COLOUR;
import lejos.hardware.Sound;

/**
 * Class to implement search and obstacle avoidance. 
 * The {@code searchVehicle} method starts the search process. 
 * It takes in the search zone boundary parameters and generates 
 * a set of waypoints to travel to. It calls the {@code moveToWaypoint} 
 * method to move to a specific way point and also checks if there are 
 * any objects in its path. If any object is detected, it is checked if 
 * it is an obstacle or a vehicle. In case of an obstacle, the 
 * {@code avoidObstacle} method is called to avoid the obstacle. 
 * In case of the vehicle, the {@code towVehicle} method
 * is called to attach the vehicle to the claw.
 * 
 * @author Mustafain
 */
public class ObjectDetection implements Runnable {

  private static final int SAFE_DIST = 5;


  /**
   * Field for instance of navigation class.
   */
  private Navigation navigation;

  /**
   * Instance of RobotClaw class.
   */
  private RobotClaw claw;

  /**
   * usLocalizer used in main.
   */
  private UltrasonicLocalization usLoc;

  /**
   * light localizer used in main.
   */
  private LightLocalization lightLoc;
  
  /*
   * Coordinates of Tunnel entrance 
   * (the center of the line in front of the entrance).
   */
  double tunX;
  double tunY;
  double tunTheta;
  
  /**
   * Search zone parameters.
   */
  private double SZ_LL_X;
  private double SZ_LL_Y;
  private double SZ_UR_X;
  private double SZ_UR_Y;

  /**
   * current direction, used to avoid obstacles 1 is +y, 2 is +x, 3 is -y, 4 is -x.
   */
  private int currDirection;
  
  /**
   * Grid next location parameters.
   */
  private double nextX;
  private double nextY;

  /**
   * Constructor for ObjectDetection Class.
   * 
   * @param navigation : Instance of Navigation Class
   * @param claw : object representing the claw of the robot
   */
  public ObjectDetection(UltrasonicLocalization us, Navigation navigation, 
                         RobotClaw claw, LightLocalization lightloc) {
    this.navigation = navigation;
    this.claw = claw;
    this.usLoc = us;
    this.lightLoc = lightloc;

  }

  /**
   * This method starts the vehicle search process. 
   * It takes in the search zone boundary parameters and generates a set
   * of way points to travel to. It calls the {@code moveToWaypoint} method 
   * to move to a specific way point and also checks if there are any objects 
   * in its path. If any object is detected, it is checked if it is an obstacle 
   * or a vehicle. In case of an obstacle, the {@code avoidObstacle} method is 
   * called to avoid the obstacle. In case of the vehicle, the {@code towVehicle} 
   * method is called to attach the vehicle to the claw.
   * 
   * @param SZ_LL_X the lower left x coordinate of the assigned search zone
   * @param SZ_LL_Y the lower left y coordinate of the assigned search zone
   * @param SZ_UR_X the upper right x coordinate of the assigned search zone
   * @param SZ_UR_Y the upper right y coordinate of the assigned search zone
   */
  public void searchVehicle(double SZ_LL_X, double SZ_LL_Y, double SZ_UR_X, double SZ_UR_Y) {
    // convert coordinates to centimeter values
    this.SZ_LL_X = SZ_LL_X * TILE_SIZE;
    this.SZ_LL_Y = SZ_LL_Y * TILE_SIZE;
    this.SZ_UR_X = SZ_UR_X * TILE_SIZE;
    this.SZ_UR_Y = SZ_UR_Y * TILE_SIZE;

    // Since this method is run after line localization is done at the exit of 
    // the tunnel, we can note down the coordinates of the entrance
    tunX = odometer.getXyt()[0];
    tunY = odometer.getXyt()[1];
    tunTheta = odometer.getXyt()[2];
    run();
  }

  /**
   * This thread runs the vehicle search algorithm. It is the thread 
   * used for navigating in a grid. It does so by visiting each tile 
   * one by one through the center. This thread is interrupted by the 
   * detectObject thread when an object is detected.
   * 
   * @author Bruno
   */
  public void run() {

    int startCorner; // Clockwise from UL =1 to LL =4
    double closestCornerX;
    double closestCornerY;
    double currX = odometer.getX();
    double currY = odometer.getY();

    // Finding the closest corner of the search zone to the robot's location
    if (Math.abs(SZ_LL_X - currX) < Math.abs(SZ_UR_X - currX)) {
      closestCornerX = SZ_LL_X;
      startCorner = 4;  // lower left
    } else {
      closestCornerX = SZ_UR_X;
      startCorner = 2;  // upper right
    }

    if (Math.abs(SZ_LL_Y - currY) < Math.abs(SZ_UR_Y - currY)) {
      closestCornerY = SZ_LL_Y;
      if (startCorner == 2) {
        startCorner = 1; // upper left
      }
    } else {
      closestCornerY = SZ_UR_Y;
      if (startCorner == 4) {
        startCorner = 3; // lower right
      }
    }

    // assumes the robot is aligned along the first line after
    // the tunnel and no object can be in that tile
    Region island = Resources.island;
    Point IL_LL = island.ll;
    Point IL_UR = island.ur;

    // Correcting the position in case the robot is on the edge of the search zone
    closestCornerX = (closestCornerX == IL_LL.x) ? closestCornerX + 0.5 * TILE_SIZE : closestCornerX;
    closestCornerX = (closestCornerX == IL_UR.x) ? closestCornerX - 0.5 * TILE_SIZE : closestCornerX;

    closestCornerY = (closestCornerY == IL_LL.y) ? closestCornerY + 0.5 * TILE_SIZE : closestCornerY;
    closestCornerY = (closestCornerY == IL_UR.y) ? closestCornerY - 0.5 * TILE_SIZE : closestCornerY;

    // Start object detection
    detectObject(Thread.currentThread());

    moveToWaypoint(closestCornerX, closestCornerY);

    /*
     * To simplify the search procedure, make the robot over to 
     * the UL corner before starting grid movement. This
     * decreases the number of possible permutations of grid movement.
     */

    if (startCorner == 2) { // upper right
      moveToWaypoint(SZ_LL_X + TILE_SIZE, SZ_UR_Y); // move to UL tile edge
      lightLoc.singleLineCorrection(0); // Localize
      moveToWaypoint(SZ_LL_X + 0.5 * TILE_SIZE, SZ_UR_Y); // Move to middle of UL tile
    }
    if (startCorner == 3) { // lower right
      // move to UR corner, middle of Tile
      moveToWaypoint(SZ_UR_X - 0.5 * TILE_SIZE, SZ_UR_Y - 0.5 * TILE_SIZE); 

      moveToWaypoint(SZ_LL_X + TILE_SIZE, SZ_UR_Y);
      lightLoc.singleLineCorrection(0);
      moveToWaypoint(SZ_LL_X + 0.5 * TILE_SIZE, SZ_UR_Y);
    }
    if (startCorner == 4) { // lower left
      moveToWaypoint(SZ_LL_X, SZ_UR_Y - TILE_SIZE);// move to UL tile edge
      lightLoc.singleLineCorrection(0); // Localize
      moveToWaypoint(SZ_LL_X, SZ_UR_Y - 0.5 * TILE_SIZE);// Move to middle of UL tile
    }

    /*
     * While loop sequentially makes robot go through the middle of 
     * each tile in the search zone. The robot stops at each line it 
     * crosses and does single line localization. It proceeds from 
     * the UL corner down to the LL corner and the robot
     * goes through each column until it reaches either the UR or LR corner.
     */
    nextX = SZ_LL_X + 0.5 * TILE_SIZE;
    nextY = SZ_UR_Y - TILE_SIZE;

    while (true) {
      while (nextY != SZ_LL_Y) {
        moveToWaypoint(nextX, nextY);
        lightLoc.singleLineCorrection(0);
        nextY -= TILE_SIZE;
      }

      // move down to mid last tile
      nextY += 0.5 * TILE_SIZE;
      moveToWaypoint(nextX, nextY);

      // If robot is not in bottom right corner, move to next x line 
      // to localize and the move to middle, i.e. 0.5 * TILE_WIDTH, 
      // of next top tile
      if (nextX + 0.5 * TILE_SIZE == SZ_UR_X) {
        break;
      }
      // Move to next x line to localize and the move to middle of next bottom tile
      nextX += 0.5 * TILE_SIZE;
      moveToWaypoint(nextX, nextY);
      lightLoc.singleLineCorrection(0);
      nextX += 0.5 * TILE_SIZE;
      moveToWaypoint(nextX, nextY);

      // Move up to top of search zone
      nextY += 0.5 * TILE_SIZE;
      while (nextY != SZ_UR_Y) {
        moveToWaypoint(nextX, nextY);
        lightLoc.singleLineCorrection(0);
        nextY += TILE_SIZE;
      }

      nextY += 0.5 * TILE_SIZE;
      moveToWaypoint(nextX, nextY);

      // If robot is not in top right corner, move to next x line 
      // to localize and the move to middle of next top tile
      if (nextX + 0.5 * TILE_SIZE == SZ_UR_X) {
        break;
      }
      nextX += 0.5 * TILE_SIZE;
      moveToWaypoint(nextX, nextY);
      lightLoc.singleLineCorrection(0);
      nextX += 0.5 * TILE_SIZE;
      moveToWaypoint(nextX, nextY);

    }
  }

  private void detectObject(final Thread trd) {

    (new Thread() {
      public void run() {
        // Due to the set up of the grid search algorithm we can 
        // assume the robot will never be under 5 cm from the map wall
        while (true) {
          int dist = usLoc.readUsDistance();
          if (dist < SAFE_DIST) {
            if (!isObstacle()) {
              towVehicle();
              // go back to tunnel entrance
              // run detector in order to evade obstacles on the way to the tunnel
              detectObject(Thread.currentThread());
              moveToWaypoint(odometer.getXyt()[0], tunY);
              moveToWaypoint(tunX, tunY);
              navigation.turnTo(tunTheta);
              NavigatorUtility.turnBy(180);
            } else {
              // Pause the thread calling object detection so that the obstacle can be avoided

              // If thread was passed from recursive call of detectObject
              if (trd != null) {
                try {
                  trd.wait();
                } catch (InterruptedException e) {
                  // Ignore
                }
              }

              avoidObstacle();
              // Re-activate the calling thread
              trd.notify();
            }
          }
        }
      }
    }).start();
  }



  /**
   * This method is called when an obstacle is detected. 
   * It avoids the obstacle by moving around the obstacle from the
   * left or right directions depending on the current position of 
   * the robot and the search zone (SZ) boundary.
   */
  public void avoidObstacle() {
    // Position of robot
    double[] odoValues = odometer.getXyt();
    double currX = odoValues[0];
    double currY = odoValues[1];

    if (currDirection == 1) { // +y direction

      if (Math.abs(currX - SZ_LL_X) > TILE_SIZE) { // is there a tile in SZ to the left?
        // then go to left tile to avoid obstacle
        moveToWaypoint(currX - TILE_SIZE, currY);
        moveToWaypoint(currX - TILE_SIZE, currY + 2 * TILE_SIZE);
        moveToWaypoint(currX, currY + 2 * TILE_SIZE);
      } else {
        // then go to right tile to avoid obstacle
        moveToWaypoint(currX + TILE_SIZE, currY);
        moveToWaypoint(currX + TILE_SIZE, currY + 2 * TILE_SIZE);
        moveToWaypoint(currX, currY + 2 * TILE_SIZE);
      }
      nextY += 2 * TILE_SIZE; // update next grid location
    }
    if (currDirection == 3) { // -y direction
      if (Math.abs(currX - SZ_LL_X) > TILE_SIZE) { // is there a tile in SZ to the left?
        // then go to left tile to avoid obstacle
        moveToWaypoint(currX - TILE_SIZE, currY);
        moveToWaypoint(currX - TILE_SIZE, currY - 2 * TILE_SIZE);
        moveToWaypoint(currX, currY - 2 * TILE_SIZE);
      } else {
        // then go to right tile to avoid obstacle
        moveToWaypoint(currX + TILE_SIZE, currY);
        moveToWaypoint(currX + TILE_SIZE, currY - 2 * TILE_SIZE);
        moveToWaypoint(currX, currY - 2 * TILE_SIZE);
      }
      nextY -= 2 * TILE_SIZE; // update next grid location
    }

    if (currDirection == 2) { // +x direction
      if (Math.abs(currY - SZ_UR_Y) > TILE_SIZE) { // is there a tile in SZ to the left?
        // then go to the tile above to avoid obstacle
        moveToWaypoint(currX, currY + TILE_SIZE);
        moveToWaypoint(currX + 2 * TILE_SIZE, currY + TILE_SIZE);
        moveToWaypoint(currX + 2 * TILE_SIZE, currY);
      } else {
        // then go to the tile below to avoid obstacle
        moveToWaypoint(currX, currY - TILE_SIZE);
        moveToWaypoint(currX + 2 * TILE_SIZE, currY - TILE_SIZE);
        moveToWaypoint(currX + 2 * TILE_SIZE, currY);
      }
      nextX += 2 * TILE_SIZE; // update next grid location
    }
    if (currDirection == 4) { // -x direction
      if (Math.abs(currY - SZ_UR_Y) > TILE_SIZE) { // is there a tile in SZ to the left?
        // then go to the tile above to avoid obstacle
        moveToWaypoint(currX, currY + TILE_SIZE);
        moveToWaypoint(currX - 2 * TILE_SIZE, currY + TILE_SIZE);
        moveToWaypoint(currX - 2 * TILE_SIZE, currY);
      } else {
        // then go to the tile below to avoid obstacle
        moveToWaypoint(currX, currY - TILE_SIZE);
        moveToWaypoint(currX - 2 * TILE_SIZE, currY - TILE_SIZE);
        moveToWaypoint(currX - 2 * TILE_SIZE, currY);
      }
      nextX -= 2 * TILE_SIZE; // update next grid location
    }

  }

  /**
   * Moves the robot to the specified waypoint.
   * 
   * @param x x-coordinate of waypoint to travel to
   * @param y y-coordinate of waypoint to travel to
   */
  public void moveToWaypoint(double x, double y) {

    double[] odoValues = odometer.getXyt();
    // subtracting odometer x and y values to estimate x and y distance needed
    double distX = x - odoValues[0];
    double distY = y - odoValues[1];

    double theta = Math.atan2(distX, distY);

    leftMotor.setSpeed(MOTOR_LOW);
    rightMotor.setSpeed(MOTOR_LOW);
    navigation.turnTo(theta);

    double hypotenuse = Math.sqrt(distY * distY + distX * distX);
    double currX = odoValues[0];
    double currY = odoValues[1];
    double distanceTravelled = 0;

    while (distanceTravelled <= hypotenuse) {
      leftMotor.setSpeed(MOTOR_LOW);
      rightMotor.setSpeed(MOTOR_LOW);
      rightMotor.forward();
      leftMotor.forward();

      if (usLoc.readUsDistance() < SAFE_DIST) {
        rightMotor.stop(true);
        leftMotor.stop(true);
        Sound.beep();
        Sound.beep();
        if (isObstacle()) {
          avoidObstacle();
          return;
        } else {
          // vehicle detected, begin rescue process
          return;
        }
      }

      double xNew = odometer.getX();
      double yNew = odometer.getY();
      distanceTravelled = Math.sqrt((xNew - currX) * (xNew - currX) 
                            + (yNew - currY) * (yNew - currY));
    }
    rightMotor.stop();
    leftMotor.stop();
  }

  /**
   * This method is used to characterize a detected object. 
   * It returns true if the detected object is an obstacle,
   * returns false otherwise.
   * 
   * @return true if obstacle is detected, false otherwise.
   */
  private boolean isObstacle() {

    // If looking at the cart
    if (LightSensorPoller.getColor() != COLOUR.WALL) {
      Resources.mediumRegulatedMotor.rotate(-CLAW_ANGLE);
      return false;
    } else { // if looking at a wall
      return true;
    }
  }

  /**
   * This method is called when a vehicle is detected, 
   * this method orients the claw to be in line with the axle. 
   * The {@code RobotClaw} class is then used to attach 
   * the vehicle to the robot.
   */
  public void towVehicle() {
    // Open claw
	  claw.openClaw();
    // move backwards to give space for turn
    NavigatorUtility.moveDistFwd((int) (-(TILE_SIZE) * 100 / 3), 200); 
    
    // rotate 180 to have claw face object
    NavigatorUtility.turnBy(180); 
    
    // move onto object
    NavigatorUtility.moveDistFwd((int) (-(TILE_SIZE) * 100 / 3), 200);
    
    // close claw
    claw.closeClaw();
  }

}
