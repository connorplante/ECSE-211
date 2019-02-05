package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigationObstacle extends Thread implements UltrasonicController {
	private int distance;
	private int filterControl;
	private static final int FILTER_OUT = 5;
	private static final int FORWARD_SPEED = 150;
	private static final double TILE_LENGTH = 30.48;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Odometer odometer;
	private double prevX; 
	private double prevY;
	private static double[] odoCurrent = new double[3];
	private static final int DISTANCE_THRESH = 20;
	private static final int CHECK_THRESH = 35;
	private static final double CORRECTION_L = 30;
	
	
	private static final double[][] MAP = { { 0.0, 2.0 }, { 1.0, 1.0 }, 
			{ 2.0, 2.0 }, { 2.0, 1.0 }, { 1.0, 0.0 } };
	
	public NavigationObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
	}
	
	@Override
	public void processUSData(int distance) {
		//System.out.println(distance);
		
		if (distance <= DISTANCE_THRESH && filterControl > FILTER_OUT) {
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
	
	public void run() {
		odometer.setXYT(0.0, 0.0, 0.0);
		
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		int i;
		for (i = 0; i < 5; i++) {
			travelTo(MAP[i][0], MAP[i][1]);
		}
	}
	
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
			if (this.distance <= DISTANCE_THRESH) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				
				navigateObstacle(flag);
				
				travelTo(x, y);
			}
		}
		
		prevX = x; 
		prevY = y;

		leftMotor.stop(true);
		rightMotor.stop(true);
	}
	
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
	
	public boolean isNavigating() {
		if (leftMotor.isMoving() || rightMotor.isMoving()) {
			return true;
		}
		return false;
	}
	
	public void navigateObstacle(boolean flag) {
		int switcher = 1;
		if (flag) {
			switcher = -1;
		}
		
		while (this.distance <= CHECK_THRESH) {
			leftMotor.rotate(switcher * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
			rightMotor.rotate(switcher * -convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
		
			System.out.print("hit1");
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L), true); // >
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L), false);
	
			leftMotor.rotate(switcher * -convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
			rightMotor.rotate(switcher * convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
			System.out.print("hit2");
		}
	    
	    leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L + 15), true); // ^
	    rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L + 15), false);
	    
//	    leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
//	    rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
//	    
//	    if (this.distance <= CHECK_THRESH) {
//	    	while (this.distance <= CHECK_THRESH)
//	    	leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
//			rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
//			
//			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L / 2), true); // ^
//		    rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, CORRECTION_L / 2), false);
//		    
//		    leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
//		    rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
//	    }
//	    System.out.print("hit3");
	    
	    leftMotor.stop(true);
		rightMotor.stop(true);
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
