package ca.mcgill.ecse211.lab4;

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
	private double x1,x2,y1,y2,thetaY,thetaX;
	
	private static final double DIST_TO_LIGHT = 13;
	
	private static final double DIFF_THRESH = -1.5;
	
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}
	
	public void run() {
		reorient();
		calculateAngles();
		
		while(UltrasonicLocalizer.isNavigating()) ;
		
		correctOdometer(); 
	}
	
	private void reorient() {
		leftMotor.forward();
		rightMotor.forward();
		
		while (UltrasonicLocalizer.isNavigating()) {
			cStatus.fetchSample(sampleCS, 0);
			float currentVal = sampleCS[0] * 100;
			
			float difference = currentVal - prevVal;
			prevVal = currentVal;
			System.out.println(difference + "\n");
			if (difference < DIFF_THRESH) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				
				break;	
			}
		}
		
		leftMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, 16), true);
		rightMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, 16), false);
		
		leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 90.0), true);
	    rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 90.0), false);
	    
	    leftMotor.forward();
		rightMotor.forward();
		
		while (UltrasonicLocalizer.isNavigating()) {
			cStatus.fetchSample(sampleCS, 0);
			float currentVal = sampleCS[0] * 100;
			
			float difference = currentVal - prevVal;
			
			prevVal = currentVal;
			
			System.out.println(difference + "\n");
			if (difference < DIFF_THRESH) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				
				break;	
			}
		}
		
		leftMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, 16), true);
		rightMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, 16), false);
	    
	}
	
	private void calculateAngles() {
		leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 270.0), true);
		rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 270.0), true);
		int lineCount = 0;
		int filterCount = 0;
		while(UltrasonicLocalizer.isNavigating()) {
			cStatus.fetchSample(sampleCS, 0);
			float currentVal = sampleCS[0] * 100;
			
			float difference = currentVal - prevVal;
			
			prevVal = currentVal;
			System.out.println(difference + "\n");
			if (difference < DIFF_THRESH && filterCount == 0) {
				Sound.beep();
				if (lineCount == 0 ) {
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

		}
		
	}
	
	private void correctOdometer() {
		thetaY = y2 - y1;
		thetaX = x2 - x1;
		System.out.println(thetaY + "\t" +thetaX);
		double x = -DIST_TO_LIGHT*Math.cos(thetaY/2.0);
		double y = -DIST_TO_LIGHT*Math.cos(thetaX/2.0);
		
		odometer.setX(x);
		odometer.setY(y);
		
		double dTheta = 270 - y2 + (thetaY / 2.0);
		
		odometer.setTheta(odometer.getXYT()[2]+dTheta);
		
	}
	
	
	/**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/** 
	 * Method to convert a distance to an arc length.
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
