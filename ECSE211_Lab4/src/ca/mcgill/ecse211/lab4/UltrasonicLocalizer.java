package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class UltrasonicLocalizer implements Runnable, UltrasonicController {
	private int distance;
	private Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean flag;
	private double alpha;
	private double beta; 
	private double theta1;
	private double theta2;

	private static final int FILTER_VAL = 1;
	private static final int D_FALLING = 20;
	private static final int D_RISING = 32;

	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}
	
	/**
	 * Method that runs the ultrasonic localization routine.
	 */
	@Override
	public void run() {
		if (flag) {
			risingEdge();
		} else {
			fallingEdge();
		}
		
	}
	
	/**
	 * Method that rotates the robot to the left and then to the right to detect both falling 
	 * edges before computing the new theta value and rotating the robot to a heading
	 * of 0 degrees.
	 */
	private void fallingEdge() {
		// turn left until falling edge is detected
		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		
		this.alpha = detectFallingEdge();
		
		leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		
		try {
			Thread.sleep(1500);
		} catch (Exception e) {
			
		}
		
		this.beta = detectFallingEdge();

		
		double dTheta = 225-(alpha+beta)/2;
		double theta = odometer.getXYT()[2];
		odometer.setTheta(theta+dTheta);
	
		turnTo(-Math.toRadians(odometer.getXYT()[2]));
		
	}
	/**
	 * Method to detect a falling edge distance while rotating. Returns the angle at which the 
	 * falling edge is detected.
	 * @return
	 */
	private double detectFallingEdge() {
		int filterCount = 0;
		while (isNavigating()) {
			if (this.distance < D_FALLING && filterCount > FILTER_VAL) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				return odometer.getXYT()[2];
			} else if (this.distance < D_FALLING) {
				filterCount++;
			}
		}
		
		return odometer.getXYT()[2];
	}
	
	/**
	 * Method that rotates the robot to the left and then to the right to detect both rising 
	 * edges before computing the new theta value and rotating the robot to a heading
	 * of 0 degrees.
	 */
	private void risingEdge() {
		// turn left until rising edge is detected
		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		
		this.alpha = detectRisingEdge();
		
		leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		
		try {
			Thread.sleep(1500);
		} catch (Exception e) {
			
		}
		
		this.beta = detectRisingEdge();

		double dTheta = 45 - (alpha + beta) / 2;
		double theta = odometer.getXYT()[2];
		odometer.setTheta(theta + dTheta);

		turnTo(-Math.toRadians(odometer.getXYT()[2]));
	}
	
	/**
	 * Method to detect a rising edge distance while rotating. Returns the angle at which the 
	 * rising edge is detected.
	 * @return
	 */
	private double detectRisingEdge() {
		int filterCount = 0;
		while (isNavigating()) {
			if (this.distance > D_RISING && filterCount > FILTER_VAL) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				return odometer.getXYT()[2];
			} else if (this.distance > D_RISING) {
				filterCount++;
			}
		}
		return odometer.getXYT()[2];
	}
	
	/**
	 * Rotates the wheels of the robot in opposite directions at the same speed to 
	 * turn the robot to face the direction of the specified theta from the origin.
	 * 
	 * The robot will minimize the rotation distance (make the smaller turn when applicable).
	 * 
	 * @param theta
	 */
	public static void turnTo(double theta) {
		// Keep the angle within the period of -pi to pi
		if (theta <= -Math.PI) {
			theta += 2 * Math.PI;
		} else if (theta > Math.PI) {
			theta -= 2 * Math.PI;
		}
		
		theta = Math.toDegrees(theta);
		
		if (theta < 0) {
			leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -theta), true);
			rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, -theta), false);
		}
		else {
			leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), true);
			rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, theta), false);
		}
		
	}
	
	
	/**
	 * Sets the distance value to that which is read by the US sensor.
	 */
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
	}

	/**
	 * Returns most recent US sensor distance reading.
	 * @return
	 */
	@Override
	public int readUSDistance() {
		return this.distance;
	}
	
	/**
	 * Sets the flag which is used to determine whether to use rising edge (true)
	 * or falling edge (false).
	 * @param flag
	 */
	public void setFlag(boolean flag) {
		this.flag = flag;
	}
	
	/**
	 * Method that returns true when the robot is moving, false if otherwise. 
	 * 
	 * @return boolean 
	 */
	public static boolean isNavigating() {
		if (leftMotor.isMoving() || rightMotor.isMoving()) {
			return true;
		}
		return false;
	}
	
	/**
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return
	   */
	public static int convertDistance(double radius, double distance) {
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
	public static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}
