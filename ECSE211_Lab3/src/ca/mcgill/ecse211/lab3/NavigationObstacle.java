package ca.mcgill.ecse211.lab3;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class NavigationObstacle extends Thread implements UltrasonicController {
	private int distance;
	private int filterControl;
	private static final int FILTER_OUT = 10;
	private static final int FORWARD_SPEED = 150;
	private static final double TILE_LENGTH = 30.48;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Odometer odometer;
	private static double[] odoCurrent = new double[3];
	private static boolean flag = false;
	
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
		
		if (distance <= 30 && filterControl > FILTER_OUT) {
			flag = true;
			navigateObstacle();
			//System.out.println(flag);
		} else if (distance <= 30) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			filterControl++;
			this.distance = distance;
		} else {
			// distance went above 30: reset filter and leave
			// distance alone.
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
		
		double dX = (x * TILE_LENGTH) - odoCurrent[0];
		double dY = (y * TILE_LENGTH) - odoCurrent[1];
		
		double dTheta = Math.atan2(dX, dY) - Math.toRadians(odoCurrent[2]);
		turnTo(dTheta);
		
		double distance = Math.hypot(dX, dY);

		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, distance), false);
		
//		while (isNavigating()) {
//			if (flag) {
//				leftMotor.stop(true);
//				rightMotor.stop(true);
//				navigateObstacle();
//			}
//		}

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
	
	public void navigateObstacle() {
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
	    rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
	    System.out.print("hit1");
	    leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20.0), true);
	    rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20.0), false);
	    
	    leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
	    rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
	    System.out.print("hit2");
	    leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20.0), true);
	    rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20.0), false);
	    
	    leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
	    rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
	    System.out.print("hit3");
	    leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20.0), true);
	    rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 20.0), false);
	    
	    leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), true);
	    rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90.0), false);
	    System.out.print("hit4");
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
