package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation extends Thread {
	private static final int FORWARD_SPEED = 150;
	private static final double TILE_LENGTH = 30.48;
	
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private static double[] odoCurrent = new double[3];
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}
	
	/**
	 * Method to begin thread execution and travel to each waypoint on the map.
	 */
	public void run() {
		odometer.setXYT(0.0, 0.0, 0.0);
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		int i;
		for (i = 0; i < 5; i++) {
			travelTo(Lab3.MAP[i][0], Lab3.MAP[i][1]);
		}
	}
	
	/** 
	 * Method to drive the robot to the specified x, y position on the grid. 
	 * 
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {
		odoCurrent = odometer.getXYT();
		
		double dX = (x * TILE_LENGTH) - odoCurrent[0];
		double dY = (y * TILE_LENGTH) - odoCurrent[1];
		
		double dTheta = Math.atan2(dX, dY) - Math.toRadians(odoCurrent[2]);
		
		turnTo(dTheta);
		
		double distance = Math.hypot(dX, dY);

		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance), false);

		leftMotor.stop(true);
		rightMotor.stop(true);
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
			leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -theta), true);
			rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, -theta), false);
		}
		else {
			leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, theta), true);
			rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, theta), false);
		}
		
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
