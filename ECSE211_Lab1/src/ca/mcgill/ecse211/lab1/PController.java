package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Controller for the Proportional implementation of the wall follower 
 * that changes motor speeds by a calculated value based on the distance read and the error observed. 
 * @author connorplante
 *
 */
public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 20;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;

	/**
	 * Constructor that initializes band variables and filter counter and sets motor speeds to begin 
	 * moving the robot forward. 
	 * @param bandCenter
	 * @param bandwidth
	 */
	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	/**
	 * Method that takes in data from the US sensor and uses said distance to 
	 * calculate the gain. The gain is then used to change motor speeds in 
	 * order to correct distance from wall.
	 * @param distance
	 */
	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//
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
		int gain = calculateGain(error);
		if (Math.abs(error) <= bandWidth) {
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Start robot moving forward
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-40);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();

		} else if (this.distance <= 12) {
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - gain); // For when robot is very close to wall
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.backward();

		} else if (error > 0) {
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED - gain+40); // too far away
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();

		} else if (error < 0) {
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // too close
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED - gain);
			WallFollowingLab.leftMotor.forward();
			WallFollowingLab.rightMotor.forward();
		}
	}
	
	private int calculateGain(int error) {
		
		int gain = Math.abs(error) * 5; 
		
		if (gain > 200) {
			gain = 100;
		}
		
		return gain; 
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

}
