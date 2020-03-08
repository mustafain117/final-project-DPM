package ca.mcgill.ecse211.project;

import static ca.mcgill.ecse211.project.Resources.odometer;
import static ca.mcgill.ecse211.project.Resources.TEXT_LCD;
import java.text.DecimalFormat;

/**
 * Class to manage the lcd display on the EV3
 */
public class Display implements Runnable{
  
  volatile boolean exit = false; 

  public void run() {
    while (!exit) { // operates continuously
      TEXT_LCD.clear();

         
      // Retrieve x, y and Theta information
      double[] position = odometer.getXyt();
      position[2] = (position[2] * 180.0 / 3.14159);

      // Print x,y,theta, and angle information
      DecimalFormat numberFormat = new DecimalFormat("######0.00");
      TEXT_LCD.drawString("X: " + numberFormat.format(position[0]), 0, 2);
      TEXT_LCD.drawString("Y: " + numberFormat.format(position[1]), 0, 3);
      TEXT_LCD.drawString("T: " + numberFormat.format(position[2]), 0, 4);
      
      try {
        Thread.sleep(1000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  public static void test() {
   TEXT_LCD.clear();
   TEXT_LCD.drawString("<<<< Test >>>>" , 0, 0);
  }
}
