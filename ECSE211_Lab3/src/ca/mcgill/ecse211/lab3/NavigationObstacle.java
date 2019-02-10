package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigationObstacle extends Thread implements UltrasonicController {
	private int distance;
	private int filterControl;
	private static final int FILTER_OUT = 5;
	private static final int FORWARD_SPEED = 120;
	private static final double TILE_LENGTH = 30.48;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Odometer odometer;
	private double prevX; 
	private double prevY;
	private static double[] odoCurrent = new double[3];
	private static final int DISTANCE_THRESH = 20;
	private static final int CHECK_THRESH = 35;
	private static final double CORRECTION_L = 30;
	
	
	public NavigationObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}
	
	/**
	 * Processes the sample reading from the ultrasonic poller and updates the distance according to the
	 * filterControl value.
	 */
	@Override
	public void processUSData(int distance) {
		
		if (distance <= DISTANCE_THRESH && filterControl > FILTER_OUT) { // robot encounters an obstacle: update distance
			this.distance = distance;
		} else if (distance <= DISTANCE_THRESH) {
			// We have repeated small values, so there might be an obstacle present
			// leave the distance alone until the obstacle presence is confirmed
			filterControl++;
		} else {
			// distance went above 30: reset filter and update the distance
			filterControl = 0;
			this.distance = distance;
		}
		
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
	 * Method to drive the robot to the specified x, y position on the grid. If the robot encounters an obstacle while moving
	 * from point A to point B, execute correction protocol. 
	 * 
	 * @param x
	 * @param y
	 */
	public void travelTo(double x, double y) {
		odoCurrent = odometer.getXYT();
		
		boolean flag = false; 
		if (x == 0 && y == 2 && prevX == 2 && prevY == 2) {
			flag = true;
		}
		
		double dX = (x * TILE_LENGTH) - odoCurrent[0];
		double dY = (y * TILE_LENGTH) - odoCurrent[1];
		
		double dTheta = Math.atan2(dX, dY) - Math.toRadians(odoCurrent[2]);
		turnTo(dTheta);
		
		double distance = Math.hypot(dX, dY);

		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance), true);

		while (isNavigating()) {
			if (this.distance <= DISTANCE_THRESH) { // distance is too low, execute correction protocol 
				leftMotor.stop(true);
				rightMotor.stop(false); //stop motors
				
				navigateObstacle(flag);
				
				travelTo(x, y); // once correction is made, recursively call travelTo() with same (x, y) to reach waypoint
			}
		}
		
		prevX = x; 
		prevY = y;

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
	 * Method which executes correction protocol when an obstacle is encountered. Takes in a boolean variable
	 * that will determine whether the robot should turn left or right in order to avoid collision.
	 * 
	 * @param flag
	 */
	public void navigateObstacle(boolean flag) {
		int switcher = 1;
		if (flag) {
			switcher = -1;
		}
		
		while (this.distance <= CHECK_THRESH) {
			leftMotor.rotate(switcher * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
			rightMotor.rotate(switcher * -convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
		
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L), true); // >
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L), false);
	
			leftMotor.rotate(switcher * -convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
			rightMotor.rotate(switcher * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
		}
	    
	    leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L+5), true); //robot is not travelling as far upward as needed
	    rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L+5), false); // therefore +5 correction is in place
	    
	    leftMotor.stop(true);
		rightMotor.stop(true);
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

	/**
	 * Method that returns the current distance.
	 */
	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
