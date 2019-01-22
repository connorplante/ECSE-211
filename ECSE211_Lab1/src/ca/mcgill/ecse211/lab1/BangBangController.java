package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

/**
 * Controller for the BangBang implementation of the wall follower 
 * that changes motor speeds by a fixed value based on the distance read. 
 * @author connorplante
 *
 */
public class BangBangController implements UltrasonicController {
	
	private static final int FILTER_OUT = 20;

	private final int bandCenter;
	private final int bandwidth;
	private final int motorLow;
	private final int motorHigh;
	private int distance;
	private int filterControl;

	/**
	 * Constructor that initializes band variables and sets motor 
	 * speeds to begin moving the robot forward
	 * @param bandCenter
	 * @param bandwidth
	 * @param motorLow
	 * @param motorHigh
	 */
	public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
		WallFollowingLab.rightMotor.setSpeed(motorHigh);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}
	
	/**
	 * Method that takes in data from the US sensor and uses said distance to 
	 * correctly change motor speeds in order to correct distance from wall.
	 * @param distance
	 */
	@Override
	public void processUSData(int distance) {
		
		if (distance >= 255 && filterControl < FILTER_OUT) {
		      // bad value, do not set the distance var, however do increment the
		      // filter value
		      filterControl++;
		    } else if (distance >= 255) {
		      // We have repeated large values, so there must actually be nothing
		      // there: leave the distance alone
		      this.distance = distance;
		    } else {
		      // distance went below 255: reset filter and leave
		      // distance alone.
		      filterControl = 0;
		      this.distance = distance;
		    }
		
		int error = this.distance - bandCenter;
		if (Math.abs(error) <= bandwidth) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
			WallFollowingLab.rightMotor.setSpeed(motorHigh-25);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();

		} else if (this.distance <= 12) {
			WallFollowingLab.leftMotor.setSpeed(motorLow+20); //For when robot is very close to wall
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.backward();
			
		} else if (error > 0) {
			WallFollowingLab.leftMotor.setSpeed(motorLow + 40); // too far away from band 
			WallFollowingLab.rightMotor.setSpeed(motorHigh);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();

		} else if (error < 0) {
			WallFollowingLab.leftMotor.setSpeed(motorHigh); // too close to band
			WallFollowingLab.rightMotor.setSpeed(motorLow);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
