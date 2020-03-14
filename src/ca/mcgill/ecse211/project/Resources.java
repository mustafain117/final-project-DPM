package ca.mcgill.ecse211.project;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
/**
 * This class is used to define static resources in one place for easy
 * access and to avoid cluttering the rest of the
 * codebase. All resources can be imported at once like this:
 * 
 * <p>{@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 * @author Mustafain, Bruno, Jasper
 *
 */
public class Resources {
  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
  /**
   * The light sensor used for colour ring detection
   */
  public static final EV3ColorSensor lightSensor = new EV3ColorSensor(SensorPort.S3);
  
  /** The right Light Sensor sensor. */
  public static final EV3ColorSensor colorSensorRight=new EV3ColorSensor(SensorPort.S4);
  
  /** The left Light Sensor sensor. */
  public static final EV3ColorSensor colorSensorLeft=new EV3ColorSensor(SensorPort.S2);

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);
  /**
   * The motor used for the claw
   */
  public static final EV3LargeRegulatedMotor mediumRegulatedMotor = new EV3LargeRegulatedMotor(MotorPort.C);

  /**
   * The LCD screen used for displaying text.
   */
  public static final TextLCD TEXT_LCD = LocalEV3.get().getTextLCD();
  
  /**
   * The odometer.
   */
  public static Odometer odometer = Odometer.getOdometer();
  /**
   * The wheel radius in centimeters.
   */
  public static final double WHEEL_RAD = 2.130;

  /**
   * The robot width in centimeters.
   */
  public static final double BASE_WIDTH = 16.03;
  /**
   * Speed of slower rotating wheel (deg/sec). Used during ultrasonic
   * localization.
   */
  public static final int MOTOR_LOW = 150;
  /**
   * Speed of the faster rotating wheel (deg/sec).
   */
  public static final int MOTOR_HIGH = 200;
  /**
   * The limit of invalid samples that we read from the US sensor before assuming no obstacle.
   */
  public static final int INVALID_SAMPLE_LIMIT = 25;
  
  /**
   * The tile size in centimeters. Note that 30.48 cm = 1 ft.
   */
  public static final double TILE_SIZE = 30.48;
  
  /**
   * Std deviation for colour sensing
   */
  public static final int STD_DEV = 3;
  
  /*
   * Std deviation for orange and yellow colours
   */
  
  public static final int STD_DEV_OY = 2;
  
  /**
   * Ideal distance between the sensor and the wall (cm).
   */
  public static final int WALL_DIST = 30;
  /**
   * Higher angle to subtract the averaged angles measured during 
   * the falling edge localization process.
   */
  public static final int THETA_HIGH_FALLING = 245;
  
  /** The number of degrees in one radian (180/PI). */
  public static final double DEG_PER_RAD = 57.2598;
  
  /** Constant for odometer conversion
   * 360/(2xPixRw)  Rw=2.8cm    (20). */
  static final int DIST_TO_DEG = 27;
  
  /**
   * Lower angle to subtract the averaged angles measured during the 
   * falling edge localization process.
   */
  public static final int THETA_LOW_FALLING = 55;
  /**
   * Initial reading from ultrasonic sensor should be larger than this value
   * because through testing, we have found that if the robot begins localization
   * facing the wall, then the errors will be high.
   * 
   * <p>This value represents an initial distance threshold that the ultrasonic sensor
   * must read before the localization process will begin.
   */
  public static final int INITIAL_DIST_THRESHOLD = 100;
  /**
   * Error value tuned to get proper turns on the wheels.
   */
  public static final double TURNING_ERROR = 19.25;
  /**
   * Wheel speed during rotation after localization is finished (deg/sec).
   */
  public static final int MOTOR_ROT = 80;
  
  /**
   * 90 degrees relative to the robot hardware. For example, 
   * calling {@code turnBy(TURN_90)} will make the robot turn 90
   * degrees.
   */
  public static final int TURN_90 = 88;

  /**
   * 180 degrees relative to the robot hardware. For example, 
   * calling {@code turnBy(TURN_180)} will make the robot turn
   * 180 degrees.
   */
  public static final int TURN_180 = 178;

  
  /**
   * Robot length in centimeters.
   */
  public static final double ROBOT_LENGTH = 4.6; 
  
/*************************************
 *                                   *
 *      COLOR RESOURCES              *
 *                                   *
 **************************************/
  
  /**
   * Blue mean RGB values
   */
  public static final double[] BLUE_MEAN = {0.248570263,0.75230169,0.610127029};
  
  /**
   * Blue standard deviation RGB values
   */
  public static final double[] BLUE_SD = {0.00278066, 0.007794649,0.005694158};
  
  /**
   * Green mean RGB values
   */
  public static final double[] GREEN_MEAN = {0.504894489,0.854782753,0.120116607};
  
  /**
   * Green standard deviation RGB values
   */
  public static final double[] GREEN_SD = {0.00288681,0.004807366,0.001653488};
  
  /**
   * Yellow mean RGB values
   */
  public static final double[] YELLOW_MEAN = {0.870748263,0.482106504,0.096802795};
  
  /**
   * Yellow standard deviation RGB values
   */
  public static final double[] YELLOW_SD = {0.009740534,0.003502987,0.002403683};
  
  /**
   * Orange mean RGB values
   */
  public static final double[] ORANGE_MEAN = {0.961721271, 0.264355713, 0.072168237};
  
  /**
   * Orange standard deviation RGB values
   */
  public static final double[] ORANGE_SD = {0.005720356, 0.001136768, 0.001012523};
  
  /**
   * Orange mean RGB values
   */
  public static final double[] WALL_MEAN = {0.63625, 0.635, 0.431875};
  
  /**
   * Orange standard deviation RGB values
   */
  public static final double[] WALL_SD = {0.04910872, 0.03425395, 0.0549204};
  
}
