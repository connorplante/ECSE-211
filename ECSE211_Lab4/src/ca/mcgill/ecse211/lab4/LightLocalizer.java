package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class LightLocalizer implements Runnable {
	private static Port portCS = LocalEV3.get().getPort("S2");
	private static SensorModes cSensor = new EV3ColorSensor(portCS);
	private static SampleProvider cStatus = cSensor.getMode("Red");
	private static float[] sampleCS = new float[cStatus.sampleSize()];

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private float prevVal = 0;
	private double x1 = -10;
	private double x2 = -10;
	private double y1 = -10;
	private double y2 = -10;
	private double thetaY,thetaX;
	
	private static final double DIST_TO_LIGHT = 12.5;
	private static final double ANGLE_CORR = 0.087; // Constant to correct underturning on final heading correction
	private static final double DIFF_THRESH = -0.030;
	
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}
	
	/** 
	 * Method that runs the light localization routine.
	 */
	public void run() {

		reorient();
		
		leftMotor.setSpeed(30);
		rightMotor.setSpeed(30);
		
		calculateAngles();
		
		while(UltrasonicLocalizer.isNavigating()) ;
		
		reachOrigin();
	}
	
	/**
	 * Method to drive the robot to a position where rotating 360 degrees around its axis will
	 * result in the light sensor detecting all four grid lines leaving the origin.
	 */
	private void reorient() {
		UltrasonicLocalizer.turnTo(Math.toRadians(45));
		
		leftMotor.forward();
		rightMotor.forward();
		float currentVal = 0;
		float difference = 0;
		while (UltrasonicLocalizer.isNavigating()) {
			currentVal = getLightReading();
			
			difference = currentVal - prevVal;
			System.out.println("\n");
			if (difference < DIFF_THRESH) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				
				break;	
			}

			prevVal = currentVal;

		}
		
		leftMotor.rotate(-UltrasonicLocalizer.convertDistance(Lab4.WHEEL_RAD, 16), true);
		rightMotor.rotate(-UltrasonicLocalizer.convertDistance(Lab4.WHEEL_RAD, 16), false);
	    
	}
	
	/**
	 * Method that rotates the robot around its driving axis to detect the four lines and 
	 * record the corresponding angles.
	 */
	private void calculateAngles() {
		boolean allLinesHit = true;
		// while loop that will try to detect all 4 lines again if any values are not set by the end of the 360.
		while (allLinesHit) {
			leftMotor.rotate(UltrasonicLocalizer.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
			rightMotor.rotate(-UltrasonicLocalizer.convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
			int lineCount = 0;
			int filterCount = 0;
			float currentVal = 0;
			float difference = 0;
			while (UltrasonicLocalizer.isNavigating()) {
				currentVal = getLightReading();

				difference = currentVal - prevVal;
				System.out.println("\n");
				if (difference < DIFF_THRESH && filterCount == 0) {
					Sound.beep();
					if (lineCount == 0) {
						x1 = odometer.getXYT()[2];
					} else if (lineCount == 1) {
						y1 = odometer.getXYT()[2];
					} else if (lineCount == 2) {
						x2 = odometer.getXYT()[2];
					} else if (lineCount == 3) {
						y2 = odometer.getXYT()[2];
					}
					filterCount++;
					lineCount++;
				} else if (difference < DIFF_THRESH && filterCount > 0) {
					filterCount++;
				} else if (difference > DIFF_THRESH) {
					filterCount = 0;
				}

				prevVal = currentVal;

			}
			if (x1 == -10 || y1 == -10 || x2 == -10 || y2 == -10) {
				x1 = -10;
				y1 = -10;
				x2 = -10;
				y2 = -10;
			} else {
				allLinesHit = false;
			}

		}
	}
	
	/**
	 * Method to calculate the correct x and y coordinates of the robot and then drive the robot to the origin
	 * before centering it at a heading of 0 degrees.
	 */
	private void reachOrigin() {
		thetaY = y2 - y1;
		thetaX = x2 - x1;
		double x = -DIST_TO_LIGHT*Math.cos(Math.toRadians(thetaY)/2.0);
		double y = -DIST_TO_LIGHT*Math.cos(Math.toRadians(thetaX)/2.0);
		
		odometer.setX(x);
		odometer.setY(y);
		
		double dTheta = 90 - y2 + 180 + (thetaY/ 2.0);
		odometer.setTheta(odometer.getXYT()[2]+dTheta);
		
		travelTo(0,0);
		
		UltrasonicLocalizer.turnTo(-Math.toRadians(odometer.getXYT()[2]) - ANGLE_CORR);
		
	}
	
	/** 
	 * Method to drive the robot to the specified x, y position on the grid. 
	 * 
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {
		double[] odoCurrent = odometer.getXYT();
		
		double dX = (x * Lab4.TILE_LENGTH) - odoCurrent[0];
		double dY = (y * Lab4.TILE_LENGTH) - odoCurrent[1];
		
		double dTheta = Math.atan2(dX, dY) - Math.toRadians(odoCurrent[2]);
		
		UltrasonicLocalizer.turnTo(dTheta);
		
		double distance = Math.hypot(dX, dY);

		leftMotor.rotate(UltrasonicLocalizer.convertDistance(Lab4.WHEEL_RAD, distance), true);
		rightMotor.rotate(UltrasonicLocalizer.convertDistance(Lab4.WHEEL_RAD, distance), false);

		leftMotor.stop(true);
		rightMotor.stop(true);
	}
	
	/**
	 * Method that returns a reading from the light sensor.
	 * @return
	 */
	private float getLightReading() {
		cStatus.fetchSample(sampleCS, 0);
		return sampleCS[0];
	}
	
}
