package ca.mcgill.ecse211.lab3;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2; // 2.2
	public static final double TRACK = 16.0; // 11.5 - 12.0

	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // No need to change
		Navigation navReg = new Navigation(leftMotor, rightMotor, odometer);
		
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	                                                              // this instance
	    float[] usData = new float[usDistance.sampleSize()];
	    
		NavigationObstacle navObst = new NavigationObstacle(leftMotor, rightMotor, odometer);
		UltrasonicPoller poller = new UltrasonicPoller(usDistance, usData, navObst);

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the motors should drive in a square or float
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString(" 	Nav	 | Nav    ", 0, 2);
			lcd.drawString("       | with   ", 0, 3);
			lcd.drawString("       | obst.  ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {

			// Display changes in position as wheels are (manually) moved

			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Thread navRegThread = new Thread(navReg);
			navRegThread.start();

		} else {
			System.out.println("\n\n\n");
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			Thread pollThread = new Thread(poller);
			pollThread.start();
			Thread navRegThread = new Thread(navReg);
			navRegThread.start();

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE) ;
		System.exit(0);
	}
}

