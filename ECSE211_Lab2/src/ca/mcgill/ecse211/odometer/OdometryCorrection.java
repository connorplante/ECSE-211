/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.lab2.SquareDriver;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 10;
	private static final double TILE_SIZE = 30.48;
	private Odometer odometer;
	private static Port portCS = LocalEV3.get().getPort("S1");
	private static SensorModes cSensor = new EV3ColorSensor(portCS);
	private static SampleProvider cStatus = cSensor.getMode("Red");
	private static float[] sampleCS = new float[cStatus.sampleSize()];
	private static final double SENSOR_OFFSET = 0.5;
	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection() throws OdometerExceptions {

		this.odometer = Odometer.getOdometer();

	}

	/**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
	odometer.setXYT(-TILE_SIZE * 0.5, -TILE_SIZE * 0.5, 0);
	long correctionStart, correctionEnd = -1000;
	float intensity = 0;
	double[] odoData = new double[3];
	double firstVal = 0.0;
	int lineCount = 0;
	int ultCount = 0;
	Sound.setVolume(25);
	System.out.println("\n\n\n\n");
	while (true) {
		correctionStart = System.currentTimeMillis();

		cStatus.fetchSample(sampleCS, 0);
		intensity = sampleCS[0];
		if (intensity < 0.2) {
			Sound.beep();
			odoData = odometer.getXYT();
			if (odoData[2] > 340 || odoData[2] < 20 || (odoData[2] > 160 && odoData[2] < 200)) {
				if (lineCount > 0) {
					if (ultCount > 6) {
						odometer.setY(-TILE_SIZE * lineCount + firstVal);
					} else {
						odometer.setY(TILE_SIZE * lineCount + firstVal);
					}
					
				} else {
					firstVal = odoData[1] - SENSOR_OFFSET;
				}
				lineCount++; 
			} else {
				if (lineCount > 0) {
					if (ultCount > 6) {
						odometer.setX(-TILE_SIZE * lineCount + firstVal);
					} else {
						odometer.setX(TILE_SIZE * lineCount + firstVal);
					}
					
				} else {
					firstVal = odoData[0] - SENSOR_OFFSET;
				}
				lineCount++; 
			}
			
			if (lineCount > 2) {
				lineCount = 0;
			}
			
			ultCount++;
			System.out.println(ultCount);
 		}

		correctionEnd = System.currentTimeMillis();
		if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
			try {
				Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
			} catch (InterruptedException e) {
				// there is nothing to be done here
			}
		}
	}
  }
}
