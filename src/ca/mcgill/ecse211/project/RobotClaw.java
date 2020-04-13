package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.CLAW_ANGLE;
import static ca.mcgill.ecse211.project.Resources.mediumRegulatedMotor;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to control the claw of the robot used 
 * for the rescue process of the vehicle, it is implemented as
 * singleton to ensure only one object of RobotClaw is created. 
 * It provides methods to open and close the claw in order
 * to attach the vehicle to the robot.
 * 
 * @author Mustafain , Bruno
 *
 */
public class RobotClaw {

  private static RobotClaw claw;

  /**
   * Motor controlling the claw.
   */
  private EV3LargeRegulatedMotor clawMotor = mediumRegulatedMotor;

  /**
   * Private constructor for RobotClaw class, implemented as Singleton.
   */
  private RobotClaw() {
    // TODO
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
    clawMotor.rotate(-CLAW_ANGLE);

  }

  /**
   * Closes the claw using the motor that the claw is attached to.
   */
  public void closeClaw() {
    clawMotor.rotate(CLAW_ANGLE);
  }

}
