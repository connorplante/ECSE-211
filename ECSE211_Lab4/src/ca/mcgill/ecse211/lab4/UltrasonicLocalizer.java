package ca.mcgill.ecse211.lab4;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class UltrasonicLocalizer implements Runnable, UltrasonicController {
	private int distance;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private boolean flag;
	private double alpha;
	private double beta; 
	private double theta1;
	private double theta2;

	private static final int D_FALLING = 20;
	private static final int D_RISING = 32;
	private static final int K = 0;

	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}
	
	@Override
	public void run() {
		if (flag) {
			risingEdge();
		} else {
			fallingEdge();
		}
		
	}
	
	private void fallingEdge() {
		// turn left until falling edge is detected
		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		
		while (isNavigating()) {
//			if (this.distance < D+K) {
//				System.out.println(this.distance);
//				this.theta1 = odometer.getXYT()[2];
//				continue;
//			} 
			if (this.distance < D_FALLING-K) {
				System.out.println(this.distance);
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				this.alpha = odometer.getXYT()[2];
			}
		}
		
		leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		
		try {
			Thread.sleep(500);
		} catch (Exception e) {
			System.out.println(e.getMessage());
		}
		
		while (isNavigating()) {
//			if (this.distance < D+K) {
//				System.out.println(this.distance);
//				this.theta1 = odometer.getXYT()[2];
//				continue;
//			} 
			if (this.distance < D_FALLING-K) {
				System.out.println(this.distance);
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				this.beta = odometer.getXYT()[2];
			}
		}
		
		double dTheta = 225-(alpha+beta)/2;
		double theta = odometer.getXYT()[2];
		odometer.setTheta(theta+dTheta);
		
		
		
		turnTo(-Math.toRadians(odometer.getXYT()[2]));
		
	}
	
	private void risingEdge() {
		// turn left until rising edge is detected
		leftMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		rightMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);

		while (isNavigating()) {
//					if (this.distance < D+K) {
//						System.out.println(this.distance);
//						this.theta1 = odometer.getXYT()[2];
//						continue;
//					} 
			if (this.distance > D_RISING - K) {
				System.out.println(this.distance);
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				this.alpha = odometer.getXYT()[2];
			}
		}

		leftMotor.rotate(convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);
		rightMotor.rotate(-convertAngle(Lab4.WHEEL_RAD, Lab4.TRACK, 360.0), true);

		try {
			Thread.sleep(500);
		} catch (Exception e) {
			System.out.println(e.getMessage());
		}

		while (isNavigating()) {
//					if (this.distance < D+K) {
//						System.out.println(this.distance);
//						this.theta1 = odometer.getXYT()[2];
//						continue;
//					} 
			if (this.distance > D_RISING - K) {
				System.out.println(this.distance);
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				this.beta = odometer.getXYT()[2];
			}
		}

		double dTheta = 45 - (alpha + beta) / 2;
		double theta = odometer.getXYT()[2];
		odometer.setTheta(theta + dTheta);

		turnTo(-Math.toRadians(odometer.getXYT()[2]));
	}
	
	/**
	 * Rotates the wheels of the robot in opposite directions at the same speed to 
	 * turn the robot to face the direction of the specified theta from the origin.
	 * 
	 * The robot will minimize the rotation distance (make the smaller turn when applicable).
	 * 
	 * @param theta
	 */
	public void turnTo(double theta) {
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
	
	@Override
	public void processUSData(int distance) {
		this.distance = distance;
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
	
	public void setFlag(boolean flag) {
		this.flag = flag;
	}
	
	/**
	 * Method that returns true when the robot is moving, false if otherwise. 
	 * 
	 * @return boolean 
	 */
	public boolean isNavigating() {
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
