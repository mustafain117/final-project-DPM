package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.TEXT_LCD;

import java.text.DecimalFormat;

/**
 * Class to manage the LCD display on the EV3.
 */
public class Display implements Runnable {

  volatile boolean exit = false;

  /**
   * Displays x, y and theta from Odometer.
   */
  public void run() {
    while (!exit) { // operates continuously
      TEXT_LCD.clear();
      double[] position = LightSensorPoller.tester(); // retrieve colors

      // Print x,y,theta, and angle information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      TEXT_LCD.drawString("R: " + numberFormat.format(position[0]), 0, 2);
      TEXT_LCD.drawString("G: " + numberFormat.format(position[1]), 0, 3);
      TEXT_LCD.drawString("B: " + numberFormat.format(position[2]), 0, 4);

      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Tests printing to the LCD.
   */
  public static void test() {
    TEXT_LCD.clear();
    TEXT_LCD.drawString("<<<< Test >>>>", 0, 0);
  }
}
