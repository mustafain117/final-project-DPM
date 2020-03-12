package ca.mcgill.ecse211.project;


/**
 * This class is used to control the claw of the robot used for the rescue process of the vehicle, it is implemented as singleton to ensure only
 * one object of RobotClaw is created. It provides methods to open and close the claw in order to attach the vehicle to the robot. 
 * @author mustafainali
 *
 */
public class RobotClaw {
  private static RobotClaw claw;
  /**
   *  It cannot be accessed externally.
   */
  private RobotClaw() {
  }

  /**
   * Returns the RobotClaw Object. Use this method to obtain an instance of RobotClaw.
   * 
   * @return the RobotClaw Object
   */
  public static RobotClaw getClaw() {
    if (claw == null) {
      claw = new RobotClaw();
    }
    return claw;
  }
  /**
   * Opens the claw using the motor that the claw is attached to.
   */
  public void openClaw() {
    
  }
  /**
   * Closes the claw using the motor that the claw is attached to.
   */
  public void closeClaw() {
    
  }
  
}
