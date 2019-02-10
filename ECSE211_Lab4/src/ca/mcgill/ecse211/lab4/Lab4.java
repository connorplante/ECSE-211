package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab4 {

	// Motor Objects, and Robot related parameters
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port lsPort = LocalEV3.get().getPort("S2");
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.2; // 2.2
	public static final double TRACK = 12.25; // 
	public static final int ROTATE_SPEED = 100;

	public static void main(String[] args) throws OdometerExceptions, InterruptedException {

		int buttonChoice;

		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd); // No need to change
		UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(leftMotor, rightMotor, odometer);
		
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	    SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	                                                              // this instance
	    float[] usData = new float[usDistance.sampleSize()];
		UltrasonicPoller poller = new UltrasonicPoller(usDistance, usData, usLocalizer);
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		do {
			// clear the display
			lcd.clear();

			// ask the user whether the UltrasonicLocalizer should use rising or falling edge
			lcd.drawString("< Left | Right >", 0, 0);
			lcd.drawString("       |        ", 0, 1);
			lcd.drawString("Rising | Falling", 0, 2);
			lcd.drawString(" Edge  |  Edge  ", 0, 3);
			lcd.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		System.out.println("\n\n\n");

		if (buttonChoice == Button.ID_LEFT) {

			// set flag to true to indicate rising edge
			usLocalizer.setFlag(true);

			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			try {
				Thread.sleep(3000);
			} catch (Exception e) {
				System.out.println(e.getMessage());
			}
			Thread pollThread = new Thread(poller);
			pollThread.start();
			Thread usLocalThread = new Thread(usLocalizer);
			usLocalThread.start();

		} else {
			
			// set flag to false to indicate falling edge
			usLocalizer.setFlag(false);
						
			Thread odoThread = new Thread(odometer);
			odoThread.start();
			Thread odoDisplayThread = new Thread(odometryDisplay);
			odoDisplayThread.start();
			try {
				Thread.sleep(3000);
			} catch (Exception e) {
				System.out.println(e.getMessage());
			}
			Thread pollThread = new Thread(poller);
			pollThread.start();
			Thread usLocalThread = new Thread(usLocalizer);
			usLocalThread.start();

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE) ;
		System.exit(0);
	}
}


