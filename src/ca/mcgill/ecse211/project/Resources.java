package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.playingfield.Point;
import ca.mcgill.ecse211.playingfield.Region;
import ca.mcgill.ecse211.wificlient.WifiConnection;
import java.math.BigDecimal;
import java.util.Map;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

/**
 * This class is used to define static resources in one place 
 * for easy access and to avoid cluttering the rest of the
 * codebase. All resources can be imported at once like this:
 * 
 * <p>{@code import static ca.mcgill.ecse211.lab3.Resources.*;}
 * 
 * @author Younes, Mustafain, Bruno, Jasper
 *
 */
public class Resources {

  /**
   * The default server IP used by the profs and TA's.
   */
  public static final String DEFAULT_SERVER_IP = "192.168.2.3";

  /**
   * The IP address of the server that transmits data to the robot. 
   * For the beta demo and competition, replace this line
   * with
   * 
   * <p>{@code public static final String SERVER_IP = DEFAULT_SERVER_IP;}
   */
  public static final String SERVER_IP = "192.168.2.3"; // = DEFAULT_SERVER_IP;

  /**
   * Your team number.
   */
  public static final int TEAM_NUMBER = 10;

  /**
   * Enables printing of debug info from the WiFi class.
   */
  public static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  /**
   * Enable this to attempt to receive Wi-Fi parameters at the start of the program.
   */
  public static final boolean RECEIVE_WIFI_PARAMS = true;


  // ------------------------ DECLARE YOUR CURRENT RESOURCES HERE -----------------------------
  // ---------------------- e.g., motors, sensors, constants, etc -----------------------------
  
  /**
   * The ultrasonic sensor.
   */
  public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
  
  /**
   * The light sensor used for colour ring detection.
   */
  public static final EV3ColorSensor lightSensor = new EV3ColorSensor(SensorPort.S3);

  /** 
   * The right Light Sensor sensor. 
   */
  public static final EV3ColorSensor colorSensorRight = new EV3ColorSensor(SensorPort.S4);

  /** 
   * The left Light Sensor sensor. 
   */
  public static final EV3ColorSensor colorSensorLeft = new EV3ColorSensor(SensorPort.S2);

  /**
   * The left motor.
   */
  public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);

  /**
   * The right motor.
   */
  public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.B);
  /**
   * The motor used for the claw.
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
   * The distance between the US sensor and the front of the vehicle.
   */
  public static final double US_OFFSET = 8;
  
  /**
   * The robot width in centimeters.
   */
  
  /**
   * The angle of rotation for the claw to open.
   */
  public static final int CLAW_ANGLE = 70;

  /**
   * The base width of the robot.
   */
  public static final double BASE_WIDTH = 16.03;
  
  /**
   * Speed of slower rotating wheel (deg/sec). Used during ultrasonic localization.
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
   * Standard deviation for colour sensing.
   */
  public static final int STD_DEV = 3;

  /*
   * Standard deviation for orange and yellow colours.
   */
  public static final int STD_DEV_OY = 2;

  /**
   * Ideal distance between the ultrasonic sensor and the wall (cm).
   */
  public static final int WALL_DIST = 30;
  
  /**
   * Higher angle to subtract the averaged angles measured during the 
   * falling edge localization process.
   */
  public static final int THETA_HIGH_FALLING = 245;

  /** 
   * The number of degrees in one radian (180/PI). 
   */
  public static final double DEG_PER_RAD = 57.2598;

  /**
   * Constant for odometer conversion 360/(2*Pi*Rw) Rw=2.8cm (20).
   */
  static final int DIST_TO_DEG = 27;

  /**
   * Lower angle to subtract the averaged angles measured 
   * during the falling edge localization process.
   */
  public static final int THETA_LOW_FALLING = 55;
  
  /**
   * Initial reading from ultrasonic sensor should be larger 
   * than this value because through testing, we have found that
   * if the robot begins localization facing the wall, then the 
   * errors will be high.
   * 
   * <p>This value represents an initial distance threshold that 
   * the ultrasonic sensor must read before the localization
   * process will begin.
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
   * 90 degrees relative to the robot hardware. For example, calling 
   * {@code turnBy(TURN_90)} will make the robot turn 90 degrees.
   */
  public static final int TURN_90 = 88;

  /**
   * 180 degrees relative to the robot hardware. For example, calling 
   * {@code turnBy(TURN_180)} will make the robot turn
   * 180 degrees.
   */
  public static final int TURN_180 = 178;

  /**
   * Robot length in centimeters.
   */
  public static final double ROBOT_LENGTH = 4.6;

  // *************************************
  // * * COLOR RESOURCES * *
  // **************************************/

  /**
   * Blue mean RGB values.
   */
  public static final double[] BLUE_MEAN = {0.248570263, 0.75230169, 0.610127029};

  /**
   * Blue standard deviation RGB values.
   */
  public static final double[] BLUE_SD = {0.00278066, 0.007794649, 0.005694158};

  /**
   * Green mean RGB values.
   */
  public static final double[] GREEN_MEAN = {0.504894489, 0.854782753, 0.120116607};

  /**
   * Green standard deviation RGB values.
   */
  public static final double[] GREEN_SD = {0.00288681, 0.004807366, 0.001653488};

  /**
   * Yellow mean RGB values.
   */
  public static final double[] YELLOW_MEAN = {0.870748263, 0.482106504, 0.096802795};

  /**
   * Yellow standard deviation RGB values.
   */
  public static final double[] YELLOW_SD = {0.009740534, 0.003502987, 0.002403683};

  /**
   * Orange mean RGB values.
   */
  public static final double[] ORANGE_MEAN = {0.961721271, 0.264355713, 0.072168237};

  /**
   * Orange standard deviation RGB values.
   */
  public static final double[] ORANGE_SD = {0.005720356, 0.001136768, 0.001012523};

  /**
   * Orange mean RGB values.
   */
  public static final double[] WALL_MEAN = {0.63625, 0.635, 0.431875};

  /**
   * Orange standard deviation RGB values.
   */
  public static final double[] WALL_SD = {0.04910872, 0.03425395, 0.0549204};

  /**
   * Container for the Wi-Fi parameters.
   */
  public static Map<String, Object> wifiParameters;

  // This static initializer MUST be declared before any Wi-Fi parameters.
  static {
    receiveWifiParameters();
  }

  /** Red team number. */
  public static int redTeam = getWP("RedTeam");

  /** Red team's starting corner. */
  public static int redCorner = getWP("RedCorner");

  /** Green team number. */
  public static int greenTeam = getWP("GreenTeam");

  /** Green team's starting corner. */
  public static int greenCorner = getWP("GreenCorner");

  /** The Red Zone. */
  public static Region red = makeRegion("Red");

  /** The Green Zone. */
  public static Region green = makeRegion("Green");

  /** The Island. */
  public static Region island = makeRegion("Island");

  /** The red tunnel footprint. */
  public static Region tnr = makeRegion("TNR");

  /** The green tunnel footprint. */
  public static Region tng = makeRegion("TNG");

  /** The red search zone. */
  public static Region szr = makeRegion("SZR");

  /** The green search zone. */
  public static Region szg = makeRegion("SZG");

  /**
   * Receives Wi-Fi parameters from the server program.
   */
  public static void receiveWifiParameters() {
    // Only initialize the parameters if needed
    if (!RECEIVE_WIFI_PARAMS || wifiParameters != null) {
      return;
    }
    System.out.println("Waiting to receive Wi-Fi parameters.");

    // Connect to server and get the data, catching any errors that might occur
    try (WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT)) {
      /*
       * getData() will connect to the server and wait until the user/TA 
       * presses the "Start" button in the GUI on their laptop with the data 
       * filled in. Once it's waiting, you can kill it by pressing the back/escape 
       * button on the EV3. getData() will throw exceptions if something goes wrong.
       */
      wifiParameters = conn.getData();
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }
  }

  /**
   * Returns the Wi-Fi parameter int value associated with the given key.
   * 
   * @param key the Wi-Fi parameter key
   * @return the Wi-Fi parameter int value associated with the given key
   */
  public static int getWP(String key) {
    if (wifiParameters != null) {
      return ((BigDecimal) wifiParameters.get(key)).intValue();
    } else {
      return 0;
    }
  }

  /**
   * Makes a point given a Wi-Fi parameter prefix.
   */
  public static Point makePoint(String paramPrefix) {
    return new Point(getWP(paramPrefix + "_x"), getWP(paramPrefix + "_y"));
  }

  /**
   * Makes a region given a Wi-Fi parameter prefix.
   */
  public static Region makeRegion(String paramPrefix) {
    return new Region(makePoint(paramPrefix + "_LL"), makePoint(paramPrefix + "_UR"));
  }

}
