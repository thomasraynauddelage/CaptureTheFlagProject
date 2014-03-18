import lejos.nxt.LCD;
import lejos.nxt.UltrasonicSensor;

/**
 * Handles the ultrasonic localization using Falling Edge detection.
 * 
 *
 */
public class USLocalizer {

	private static final double ROTATION_SPEED = 30;
	private Odometer odo;
	private double currentAng;
	private TwoWheeledRobot robot;
	//private UltrasonicSensor us; // use for us.getFilteredData() if necessary
	private USPoller usPoller;
	public int distance;
	private double deltaT;
	private double newAngle;

/**Constructor for the USLocalizer
 * 
 * @param odo the odometer
 * @param usPoller the usPoller
 */
	public USLocalizer(Odometer odo, USPoller usPoller) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.usPoller = usPoller;

		// switch off the ultrasonic sensor
		//us.off();
	}

	/**
	 * Performs an ultrasonic localization using falling edge detection (i.e. most of the time facing opposite the wall). 
	 * At the end of the localization the robot will be facing at an angle of 45 degrees.
	 */
	public void doUSLocalization() {
		double [] pos = new double [3];
		double angleA =0.0, angleB =0.0;
		// rotate the robot until it sees no wall
		distance = usPoller.getFilteredData();
		if (distance < 50 ){ // if facing a wall
			while(distance < 50 ){ // while facing a wall
				robot.setRotationSpeed(ROTATION_SPEED);
				distance = usPoller.getFilteredData();
			}
		}
		// keep rotating until the robot sees a wall, then latch the angle

		while(distance ==50){ // while not facing a wall
			robot.setRotationSpeed(ROTATION_SPEED);
			distance = usPoller.getFilteredData();
		}
		angleA = Math.abs(odo.getAng() - 360); // gets the bottom wall heading of robot 
		LCD.drawString("a:   " + angleA, 0, 6);
		// switch direction and wait until it sees no wall

		while (distance < 50){ // while facing wall
			robot.setRotationSpeed(-ROTATION_SPEED);
			distance = usPoller.getFilteredData();
		}
		try { Thread.sleep(1000); } catch (InterruptedException e) {}
		// keep rotating until the robot sees a wall, then latch the angle

		if (distance == 50) { // if not facing wall
			while (distance == 50) { // while not facing wall
				robot.setRotationSpeed(-ROTATION_SPEED);
				distance = usPoller.getFilteredData();
			}
			angleB = Math.abs(odo.getAng() - 360); // gets the left wall heading of the robot
			LCD.drawString("b:   " + angleB, 0, 6);
		}

		// angleA is clockwise from angleB, so assume the average of the
		// angles to the right of angleB is 45 degrees past 'north'

		// calculates the corrective angle
		if (angleA < angleB) { 
			deltaT = 45 - (angleA + angleB)/2;
		}
		else {
			deltaT = 225 - (angleA + angleB)/2;
		}

		currentAng = odo.getAng();
		newAngle =deltaT - 45;
		robot.turnTo(Odometer.minimumAngleFromTo(currentAng, newAngle));
		robot.setSpeeds(0.0, 0.0);

		odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
	}

}